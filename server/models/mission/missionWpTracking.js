import { eventBus, EVENTS } from '../../common/eventBus.js';
import { missionLogger as logger } from '../../common/logger.js';
import { ROUTE_STATUS } from '../../config/status.js';
import { haversineMeters } from '../../common/geo.js';
import { missionModel } from './mission.js';
import {
  signalFlightState,
  signalAutopilotFeedback,
  signalTimeEstimate,
  signalDeviation,
  combineSignals,
  clearDeviationHistory,
} from './missionSignals.js';

const WP_REACHED_THRESHOLD_M = 2;
const TRACKABLE_STATUSES = [ROUTE_STATUS.COMMANDED, ROUTE_STATUS.RUNNING];

// Diagnostics-only updates (no WP/status change) are capped to this interval per
// device. Anomalies/estimates can hold steady for many consecutive ticks and the
// GPS feed runs at 20-40Hz; without this cap a persistent anomaly would broadcast
// at feed rate instead of at a UI-relevant rate.
const SIGNAL_EMIT_MIN_INTERVAL_MS = 1500;

// In-memory registry of routes currently being tracked, keyed by deviceId. Populated
// reactively from ROUTE_UPDATED — mission.js never calls into this module directly,
// it just emits (as it already does for every create/edit); this module listens.
// That keeps the dependency one-directional (missionWpTracking → mission.js) and
// avoids a circular import. Bootstrapped once at startup from the DB (see init()) so
// routes already in flight survive a server restart.
const _tracked = new Map();
const _lastSignalEmit = new Map();

export class missionWpTracking {
  /**
   * Wires the reactive subscriptions and rehydrates in-flight routes from the DB.
   * Call once at server startup (see server.js), after the DB is ready.
   */
  static async init() {
    eventBus.onSafe(EVENTS.ROUTE_UPDATED, (route) => this._onRouteUpdated(route));
    eventBus.onSafe(EVENTS.POSITION_RECEIVED, (position) => this.checkProgress(position.deviceId, position));

    const routes = await missionModel.getActiveRoutes();
    for (const route of routes) {
      await this._track(route.get({ plain: true }));
    }
    logger.info(`missionWpTracking: bootstrap tracked ${routes.length} active route(s)`);
  }

  static async _onRouteUpdated(route) {
    if (TRACKABLE_STATUSES.includes(route.status)) {
      await this._track(route);
    } else {
      this._untrack(route.deviceId);
    }
  }

  // Refreshes the cheap fields (currentWp/totalWp/initTime/status) when the same
  // route is already tracked. Resolves mission → plan → waypoints ONCE per route
  // lifecycle — when a device starts being tracked or switches to a different route —
  // instead of on every position tick.
  static async _track(route) {
    const existing = _tracked.get(route.deviceId);
    if (existing && existing.routeId === route.id) {
      existing.status = route.status;
      existing.currentWp = route.currentWp ?? existing.currentWp;
      existing.totalWp = route.totalWp ?? existing.totalWp;
      existing.initTime = route.initTime ?? existing.initTime;
      return;
    }

    const mission = await missionModel.getMissionValue(route.missionId);
    if (!mission?.planId) return;
    const plan = await missionModel.getMissionPlan(mission.planId);
    if (!plan?.missionData?.route) return;

    const { devicesController } = await import('../../controllers/devices.js');
    const devRoute = await this._findRouteForDevice(plan.missionData, route.deviceId, devicesController);
    if (!devRoute?.wp?.length) return;

    _tracked.set(route.deviceId, {
      routeId: route.id,
      missionId: route.missionId,
      deviceId: route.deviceId,
      status: route.status,
      currentWp: route.currentWp ?? 0,
      totalWp: route.totalWp ?? devRoute.wp.length,
      initTime: route.initTime,
      waypoints: devRoute.wp,
      attributes: devRoute.attributes,
    });
    logger.debug(`WpTracking: now tracking device=${route.deviceId} route=${route.id}`);
  }

  static _untrack(deviceId) {
    if (!_tracked.delete(deviceId)) return;
    _lastSignalEmit.delete(deviceId);
    clearDeviationHistory(deviceId);
    logger.debug(`WpTracking: stopped tracking device=${deviceId}`);
  }

  /**
   * Called on every position update that has lat/lon (via POSITION_RECEIVED). Pure
   * in-memory lookup — no DB access for devices without an active route (the common
   * case), and none for the diagnostics-only path either (see below).
   */
  static async checkProgress(deviceId, position) {
    if (!position?.latitude || !position?.longitude) return;

    const tracked = _tracked.get(deviceId);
    if (!tracked) return;

    const { waypoints, attributes, currentWp, totalWp, initTime, routeId, missionId, status } = tracked;
    if (currentWp >= waypoints.length) return;

    const target = waypoints[currentWp];
    const [tLat, tLon] = Array.isArray(target.pos) ? target.pos : [target.pos.lat, target.pos.lon];
    const dist = haversineMeters(position.latitude, position.longitude, tLat, tLon);

    // --- Run all signals in parallel ---
    const signals = [
      signalFlightState(deviceId),
      signalAutopilotFeedback(deviceId, totalWp),
      signalTimeEstimate(waypoints, attributes, initTime, currentWp, totalWp),
      signalDeviation(deviceId, position.latitude, position.longitude, target),
    ];
    const { wpEstimate, confidence, anomalies } = combineSignals(signals);

    logger.debug(
      `WpTracking device=${deviceId} wp=${currentWp}/${waypoints.length} dist=${dist.toFixed(1)}m ` +
        `signals={estimate:${wpEstimate},confidence:${confidence},anomalies:[${anomalies}]}`
    );

    // If a high-confidence signal (autopilot feedback) gives a higher WP, advance directly
    if (wpEstimate !== null && confidence === 'high' && wpEstimate > currentWp) {
      const jumpTarget = Math.min(wpEstimate, waypoints.length);
      const isLast = jumpTarget >= waypoints.length;
      await missionModel.editRoute(
        {
          id: routeId,
          currentWp: jumpTarget,
          status: isLast ? ROUTE_STATUS.COMPLETED : ROUTE_STATUS.RUNNING,
          endTime: isLast ? new Date() : undefined,
        },
        { anomalies, wpEstimate, confidence }
      );
      logger.info(`WpTracking device=${deviceId} autopilot feedback jump wp ${currentWp} → ${jumpTarget}`);
      return;
    }

    // Haversine: UAV reached current WP
    if (dist <= WP_REACHED_THRESHOLD_M) {
      const nextWp = currentWp + 1;
      const isLast = nextWp >= waypoints.length;

      await missionModel.editRoute(
        {
          id: routeId,
          currentWp: nextWp,
          status: isLast ? ROUTE_STATUS.COMPLETED : ROUTE_STATUS.RUNNING,
          endTime: isLast ? new Date() : undefined,
        },
        { anomalies, wpEstimate, confidence }
      );

      logger.info(`WpTracking device=${deviceId} wp reached ${currentWp} → ${nextWp} last=${isLast}`);
      return;
    }

    // No WP advance — nothing persisted changes, so emit straight from the cached
    // registry entry (no DB round-trip), throttled so a sustained anomaly/estimate
    // doesn't broadcast at GPS-feed rate.
    const hasNewInfo = anomalies.length > 0 || (wpEstimate !== null && wpEstimate !== currentWp);
    if (!hasNewInfo) return;

    const lastEmit = _lastSignalEmit.get(deviceId) ?? 0;
    if (Date.now() - lastEmit < SIGNAL_EMIT_MIN_INTERVAL_MS) return;
    _lastSignalEmit.set(deviceId, Date.now());

    missionModel.emitRouteSignals(
      { id: routeId, missionId, deviceId, status, currentWp, totalWp },
      { anomalies, wpEstimate, confidence }
    );
  }

  static async _findRouteForDevice(missionData, deviceId, devicesController) {
    for (const r of missionData.route ?? []) {
      const device = await devicesController.getByName(r.uav);
      if (device?.id === deviceId) return r;
    }
    return null;
  }
}
