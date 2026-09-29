import { deviceSM } from './missionExecutionSM.js';
import { createActor } from 'xstate';
import { missionController } from '../../controllers/mission.js';
import { missionLogger as logger } from '../../common/logger.js';
import { eventBus, EVENTS } from '../../common/eventBus.js';

const listSM = {}; // state-machine actors keyed by taskId (a device can run several tasks per mission)

export class missionSMModel {
  /**
   * @param {{ alreadyRunning?: boolean }} opts - true for a task that was loaded and
   *   commanded outside the machine (manual flow): it starts in RunningMission.
   */
  static createActorMission(uavId, missionId, taskId, { alreadyRunning = false } = {}) {
    let released = false;
    listSM[taskId] = createActor(deviceSM).start();
    listSM[taskId].subscribe((state) => {
      logger.debug(`State machine task=${taskId} state=${state.value} context=${JSON.stringify(state.context)}`);
      missionController.updateMission({
        device: state.context.uavId,
        mission: state.context.missionId,
        state: state.value,
      });
      // END lingers a few seconds before the actor is deleted; the device is free from here.
      if (state.value === 'END' && !released) {
        released = true;
        eventBus.emitSafe(EVENTS.TASK_DEVICE_RELEASED, { deviceId: uavId, missionId, taskId });
      }
    });
    listSM[taskId].send({ type: alreadyRunning ? 'AttachRunning' : 'ChangeId', value: { uavId, missionId, taskId } });
    return true;
  }

  static hasActor(taskId) {
    return listSM.hasOwnProperty(taskId);
  }

  static deviceHasActorInFlight(deviceId) {
    return Object.values(listSM).some((actor) => {
      const snapshot = actor.getSnapshot();
      return snapshot.context.uavId === deviceId && snapshot.value !== 'END';
    });
  }

  // ROS end-of-mission signals carry only the device. Resolve them through the actors,
  // not the DB: by the time the signal arrives wpTracking may already have marked the
  // task COMPLETED. At most one actor per device can be in a given in-flight state.
  static _taskIdForDevice(deviceId, stateValue) {
    const entry = Object.entries(listSM).find(([, actor]) => {
      const snapshot = actor.getSnapshot();
      return snapshot.context.uavId === deviceId && snapshot.value === stateValue;
    });
    return entry ? Number(entry[0]) : null;
  }

  static deviceFinishedMission(deviceId) {
    const taskId = this._taskIdForDevice(deviceId, 'RunningMission');
    if (taskId == null) {
      logger.warn(`No running task state machine for device id=${deviceId}`);
      return false;
    }
    listSM[taskId].send({ type: 'downloadFilesUAV' });
    return true;
  }

  static deviceSyncedFiles(deviceId) {
    const taskId = this._taskIdForDevice(deviceId, 'UAVDownloadFiles');
    if (taskId == null) {
      logger.warn(`No task downloading files for device id=${deviceId}`);
      return false;
    }
    listSM[taskId].send({ type: 'downloadFilesGCS' });
    return true;
  }

  static DeleteActor(taskId) {
    if (listSM.hasOwnProperty(taskId)) {
      listSM[taskId].stop();
      delete listSM[taskId];
      logger.info(`State machine deleted for task id=${taskId}`);
    } else {
      logger.warn(`No state machine for task id=${taskId}`);
    }
  }
}
