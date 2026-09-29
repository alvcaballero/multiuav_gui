import { eventBus, EVENTS } from '../../common/eventBus.js';
import { missionLogger as logger } from '../../common/logger.js';
import {
  MISSION_STATUS,
  TASK_STATUS,
  TASK_ACTIVE_STATUS,
  TASK_TERMINAL_STATUS,
  TASK_FAILED_STATUS,
} from '../../config/status.js';
import { missionModel } from './mission.js';
import { missionSMModel } from './missionSM.js';
import { readyTasks, descendants } from './taskGraph.js';

// A dependency is satisfied once the task reached its last waypoint.
const DEPENDENCY_DONE = [TASK_STATUS.COMPLETED, TASK_STATUS.END];

// Per-mission job queue. Every read-decide-dispatch runs alone for its mission, so
// two events arriving together (two parents completing at once) can't both see the
// child as pending and dispatch it twice.
const _queues = new Map();

const toGraph = (tasks) => tasks.map((t) => ({ task_id: t.taskKey, depends_on: t.dependsOn ?? [], device: t.deviceId }));

export class taskScheduler {
  /** Call once at server startup (see server.js). */
  static init() {
    eventBus.onSafe(EVENTS.TASK_UPDATED, (task) => this._onTaskUpdated(task));
    eventBus.onSafe(EVENTS.TASK_DEVICE_RELEASED, ({ deviceId }) => this._onDeviceReleased(deviceId));
  }

  /**
   * Dispatches every task of a RUNNING mission whose dependencies are done and whose
   * device is free. Safe to call any number of times. Resolves to the dispatched taskKeys.
   */
  static dispatchReady(missionId) {
    return this._serialize(missionId, () => this._dispatch(missionId));
  }

  static async _onTaskUpdated(task) {
    if (!TASK_TERMINAL_STATUS.includes(task.status) || task.taskKey == null) return;
    if (TASK_FAILED_STATUS.includes(task.status)) {
      await this._serialize(task.missionId, () => this._skipDependents(task.missionId, task.taskKey));
    }
    await this.dispatchReady(task.missionId);
    // A task run without a state machine (manual-flow roots) frees its device here:
    // no TASK_DEVICE_RELEASED will come for it.
    if (task.deviceId != null && !missionSMModel.deviceHasActorInFlight(task.deviceId)) {
      await this._onDeviceReleased(task.deviceId);
    }
  }

  // A device freed by one mission may be what another task (of any mission) waits for.
  static async _onDeviceReleased(deviceId) {
    const waiting = await missionModel.getTasks({ deviceId, status: TASK_STATUS.INIT });
    const missionIds = [...new Set(waiting.map((t) => t.missionId))];
    await Promise.all(missionIds.map((id) => this.dispatchReady(id)));
  }

  static async _dispatch(missionId) {
    const mission = await missionModel.getMissionValue(missionId);
    // The gate: manual missions only start flowing once commanded (RUNNING).
    if (mission?.status !== MISSION_STATUS.RUNNING) return [];

    const tasks = await missionModel.getTasks({ missionId });
    const byKey = new Map(tasks.map((t) => [t.taskKey, t]));
    const pending = new Set(
      tasks.filter((t) => t.status === TASK_STATUS.INIT && !missionSMModel.hasActor(t.id)).map((t) => t.taskKey)
    );
    const done = new Set(tasks.filter((t) => DEPENDENCY_DONE.includes(t.status)).map((t) => t.taskKey));

    const dispatched = [];
    for (const node of readyTasks(toGraph(tasks), { pending, done })) {
      const task = byKey.get(node.task_id);
      if (await this._deviceBusy(task.deviceId, task.id)) {
        logger.debug(`Scheduler: task ${task.taskKey} of mission ${missionId} ready, waiting for device ${task.deviceId}`);
        continue;
      }
      missionSMModel.createActorMission(task.deviceId, missionId, task.id);
      dispatched.push(task.taskKey);
      logger.info(`Scheduler: dispatched task ${task.taskKey} (id=${task.id}) of mission ${missionId}`);
    }
    return dispatched;
  }

  // Reaching the last waypoint satisfies dependents but doesn't free the device: it
  // may still run its end-of-mission action (RTL/land) until its state machine ends.
  static async _deviceBusy(deviceId, taskId) {
    if (missionSMModel.deviceHasActorInFlight(deviceId)) return true;
    const active = await missionModel.getTasks({ deviceId, status: TASK_ACTIVE_STATUS });
    return active.some((t) => t.id !== taskId);
  }

  static async _skipDependents(missionId, taskKey) {
    const tasks = await missionModel.getTasks({ missionId });
    const downstream = new Set(descendants(toGraph(tasks), taskKey));
    for (const task of tasks) {
      if (!downstream.has(task.taskKey) || task.status !== TASK_STATUS.INIT) continue;
      await missionModel.editTask({
        id: task.id,
        status: TASK_STATUS.SKIPPED,
        errorMessage: `No se ejecutó: la tarea ${taskKey} de la que depende no se completó`,
      });
    }
  }

  static _serialize(missionId, job) {
    const previous = _queues.get(missionId) ?? Promise.resolve();
    const run = previous.then(job);
    const settled = run.catch((err) => logger.error(`Scheduler: mission ${missionId} job failed: ${err.message}`));
    _queues.set(missionId, settled);
    settled.then(() => {
      if (_queues.get(missionId) === settled) _queues.delete(missionId);
    });
    return run;
  }
}
