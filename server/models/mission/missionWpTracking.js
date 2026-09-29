import { eventBus, EVENTS } from '../../common/eventBus.js';
import { missionLogger as logger } from '../../common/logger.js';
import { TASK_STATUS } from '../../config/status.js';
import { haversineMeters } from '../../common/geo.js';
import { missionModel } from './mission.js';
import { normalizeMission } from './taskGraph.js';
import {
  signalFlightState,
  signalAutopilotFeedback,
  signalTimeEstimate,
  signalDeviation,
  combineSignals,
  clearDeviationHistory,
} from './missionSignals.js';

const WP_REACHED_THRESHOLD_M = 2;
const TRACKABLE_STATUSES = [TASK_STATUS.COMMANDED, TASK_STATUS.RUNNING];

// Diagnostics-only updates (no WP/status change) are capped to this interval per
// device. Anomalies/estimates can hold steady for many consecutive ticks and the
// GPS feed runs at 20-40Hz; without this cap a persistent anomaly would broadcast
// at feed rate instead of at a UI-relevant rate.
const SIGNAL_EMIT_MIN_INTERVAL_MS = 1500;

// In-memory registry of routes currently being tracked, keyed by deviceId. Populated
// reactively from TASK_UPDATED — mission.js never calls into this module directly,
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
    eventBus.onSafe(EVENTS.TASK_UPDATED, (task) => this._onTaskUpdated(task));
    eventBus.onSafe(EVENTS.POSITION_RECEIVED, (position) => this.checkProgress(position.deviceId, position));

    const tasks = await missionModel.getTrackableTasks();
    for (const task of tasks) {
      await this._track(task.get({ plain: true }));
    }
    logger.info(`missionWpTracking: bootstrap tracked ${tasks.length} active task(s)`);
  }

  // A device can own several tasks of one mission, so an update for one of its OTHER
  // tasks (e.g. a later task being skipped) must not untrack the one in flight.
  static async _onTaskUpdated(task) {
    if (TRACKABLE_STATUSES.includes(task.status)) {
      await this._track(task);
    } else if (_tracked.get(task.deviceId)?.taskId === task.id) {
      this._untrack(task.deviceId);
    }
  }

  // Refreshes the cheap fields (currentWp/totalWp/initTime/status) when the same
  // task is already tracked. Resolves mission → plan → waypoints ONCE per task
  // lifecycle — when a device starts being tracked or switches to a different task —
  // instead of on every position tick.
  static async _track(task) {
    const existing = _tracked.get(task.deviceId);
    if (existing && existing.taskId === task.id) {
      existing.status = task.status;
      existing.currentWp = task.currentWp ?? existing.currentWp;
      existing.totalWp = task.totalWp ?? existing.totalWp;
      existing.initTime = task.initTime ?? existing.initTime;
      return;
    }

    const mission = await missionModel.getMissionValue(task.missionId);
    if (!mission?.planId) return;
    const plan = await missionModel.getMissionPlan(mission.planId);
    if (!plan?.missionData) return;

    let taskDef;
    try {
      taskDef = normalizeMission(plan.missionData).tasks.find((t) => t.task_id === task.taskKey);
    } catch (err) {
      logger.warn(`WpTracking: plan ${plan.id} is not a valid task graph, not tracking task ${task.id}: ${err.message}`);
      return;
    }
    if (!taskDef?.wp?.length) return;

    _tracked.set(task.deviceId, {
      taskId: task.id,
      missionId: task.missionId,
      deviceId: task.deviceId,
      status: task.status,
      currentWp: task.currentWp ?? 0,
      totalWp: task.totalWp ?? taskDef.wp.length,
      initTime: task.initTime,
      waypoints: taskDef.wp,
      params: taskDef.params,
    });
    logger.debug(`WpTracking: now tracking device=${task.deviceId} task=${task.id} (${task.taskKey})`);
  }

  static _untrack(deviceId) {
    if (!_tracked.delete(deviceId)) return;
    _lastSignalEmit.delete(deviceId);
    clearDeviationHistory(deviceId);
    logger.debug(`WpTracking: stopped tracking device=${deviceId}`);
  }

  /**
   * Called on every position update that has lat/lon (via POSITION_RECEIVED). Pure
   * in-memory lookup — no DB access for devices without an active task (the common
   * case), and none for the diagnostics-only path either (see below).
   */
  static async checkProgress(deviceId, position) {
    if (!position?.latitude || !position?.longitude) return;

    const tracked = _tracked.get(deviceId);
    if (!tracked) return;

    const { waypoints, params, currentWp, totalWp, initTime, taskId, missionId, status } = tracked;
    if (currentWp >= waypoints.length) return;

    const target = waypoints[currentWp];
    const [tLat, tLon] = Array.isArray(target.pos) ? target.pos : [target.pos.lat, target.pos.lon];
    const dist = haversineMeters(position.latitude, position.longitude, tLat, tLon);

    // --- Run all signals in parallel ---
    const signals = [
      signalFlightState(deviceId),
      signalAutopilotFeedback(deviceId, totalWp),
      signalTimeEstimate(waypoints, params, initTime, currentWp, totalWp),
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
      await missionModel.editTask(
        {
          id: taskId,
          currentWp: jumpTarget,
          status: isLast ? TASK_STATUS.COMPLETED : TASK_STATUS.RUNNING,
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

      await missionModel.editTask(
        {
          id: taskId,
          currentWp: nextWp,
          status: isLast ? TASK_STATUS.COMPLETED : TASK_STATUS.RUNNING,
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

    missionModel.emitTaskSignals(
      { id: taskId, missionId, deviceId, status, currentWp, totalWp },
      { anomalies, wpEstimate, confidence }
    );
  }

}
