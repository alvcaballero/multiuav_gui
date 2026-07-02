import sequelize from '../../common/sequelize.js';
import { eventBus, EVENTS } from '../../common/eventBus.js';
import { missionLogger as logger } from '../../common/logger.js';
import { ROUTE_STATUS, MISSION_STATUS } from '../../config/status.js';
import {
  signalFlightState,
  signalAutopilotFeedback,
  signalTimeEstimate,
  signalDeviation,
  combineSignals,
  clearDeviationHistory,
} from './missionSignals.js';

const WP_REACHED_THRESHOLD_M = 2;

function haversineMeters(lat1, lon1, lat2, lon2) {
  const R = 6371000;
  const toRad = (d) => (d * Math.PI) / 180;
  const dLat = toRad(lat2 - lat1);
  const dLon = toRad(lon2 - lon1);
  const a = Math.sin(dLat / 2) ** 2 + Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) ** 2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

export class missionWpTracking {
  /**
   * Called on every position update that has lat/lon.
   * Runs all tracking signals in parallel and emits combined progress.
   */
  static async checkProgress(deviceId, position) {
    if (!position?.latitude || !position?.longitude) return;

    let missionRoute;
    try {
      missionRoute = await sequelize.models.MissionRoute.findOne({
        where: { deviceId, status: [ROUTE_STATUS.COMMANDED, ROUTE_STATUS.RUNNING] },
      });
    } catch (err) {
      logger.error(`WpTracking: DB error device=${deviceId}: ${err.message}`);
      return;
    }

    if (!missionRoute) return;

    const mission = await sequelize.models.Mission.findOne({ where: { id: missionRoute.missionId } });
    if (!mission?.planId) return;

    const plan = await sequelize.models.MissionPlan.findOne({ where: { id: mission.planId } });
    if (!plan?.missionData?.route) return;

    const { devicesController } = await import('../../controllers/devices.js');
    const devRoute = await this._findRouteForDevice(plan.missionData, deviceId, devicesController);
    if (!devRoute?.wp?.length) return;

    const waypoints = devRoute.wp;
    const currentWp = missionRoute.currentWp ?? 0;
    if (currentWp >= waypoints.length) return;

    const target = waypoints[currentWp];
    const [tLat, tLon] = Array.isArray(target.pos) ? target.pos : [target.pos.lat, target.pos.lon];
    const dist = haversineMeters(position.latitude, position.longitude, tLat, tLon);

    // --- Run all signals in parallel ---
    const signals = [
      signalFlightState(deviceId),
      signalAutopilotFeedback(deviceId, missionRoute.totalWp),
      signalTimeEstimate(waypoints, devRoute.attributes, missionRoute.initTime, currentWp, missionRoute.totalWp),
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
      missionRoute.currentWp = jumpTarget;
      missionRoute.status = isLast ? ROUTE_STATUS.COMPLETED : ROUTE_STATUS.RUNNING;
      if (isLast) missionRoute.endTime = new Date();
      await missionRoute.save();
      logger.info(`WpTracking device=${deviceId} autopilot feedback jump wp ${currentWp} → ${jumpTarget}`);
      this._emitProgress(missionRoute, deviceId, jumpTarget, anomalies);
      if (isLast) await this._checkMissionComplete(missionRoute.missionId);
      return;
    }

    // Haversine: UAV reached current WP
    if (dist <= WP_REACHED_THRESHOLD_M) {
      const nextWp = currentWp + 1;
      const isLast = nextWp >= waypoints.length;

      missionRoute.currentWp = nextWp;
      missionRoute.status = isLast ? ROUTE_STATUS.COMPLETED : ROUTE_STATUS.RUNNING;
      if (isLast) {
        missionRoute.endTime = new Date();
        clearDeviationHistory(deviceId);
      }
      await missionRoute.save();

      logger.info(`WpTracking device=${deviceId} wp reached ${currentWp} → ${nextWp} last=${isLast}`);
      this._emitProgress(missionRoute, deviceId, nextWp, anomalies);
      if (isLast) await this._checkMissionComplete(missionRoute.missionId);
      return;
    }

    // No WP advance — but still emit if there are anomalies or time estimate differs
    const hasNewInfo = anomalies.length > 0 || (wpEstimate !== null && wpEstimate !== currentWp);
    if (hasNewInfo) {
      this._emitProgress(missionRoute, deviceId, currentWp, anomalies, { wpEstimate, confidence });
    }
  }

  static _emitProgress(missionRoute, deviceId, currentWp, anomalies, signals = {}) {
    eventBus.emitSafe(EVENTS.MISSION_PROGRESS, {
      missionId: missionRoute.missionId,
      deviceId,
      routeId: missionRoute.id,
      currentWp,
      totalWp: missionRoute.totalWp,
      completed: missionRoute.status === ROUTE_STATUS.COMPLETED,
      anomalies,                          // e.g. ['DEVIATION', 'RTH_SUSPECTED']
      wpEstimate: signals.wpEstimate ?? null,
      confidence: signals.confidence ?? null,
    });
  }

  static async _findRouteForDevice(missionData, deviceId, devicesController) {
    for (const r of missionData.route ?? []) {
      const device = await devicesController.getByName(r.uav);
      if (device?.id === deviceId) return r;
    }
    return null;
  }

  static async _checkMissionComplete(missionId) {
    const routes = await sequelize.models.MissionRoute.findAll({ where: { missionId } });

    // A route in ERROR never flew — it doesn't block completion, but it means the
    // mission finished with failures. Only the "active" (non-error) routes need to
    // be COMPLETED for the mission to be considered done.
    const active = routes.filter((r) => r.status !== ROUTE_STATUS.ERROR);
    const hasErrors = active.length < routes.length;

    // If every route errored there is nothing to complete (mission already ERROR).
    if (active.length === 0) return;
    if (!active.every((r) => r.status === ROUTE_STATUS.COMPLETED)) return;

    const status = hasErrors ? MISSION_STATUS.COMPLETED_WITH_ERRORS : MISSION_STATUS.COMPLETED;
    await sequelize.models.Mission.update(
      { status, endTime: new Date() },
      { where: { id: missionId } }
    );
    logger.info(`WpTracking: Mission ${missionId} finished status=${status} (errors=${hasErrors})`);
    eventBus.emitSafe(EVENTS.MISSION_COMPLETED, { missionId, status });
  }
}
