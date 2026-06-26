import { machine } from './missionExecutionSM.js';
import { createActor } from 'xstate';
import { missionController } from '../../controllers/mission.js';
import logger from '../../common/logger.js';

const listSM = {}; // lista de acots maquinas de estados por id de UAV

export class missionSMModel {
  static createActorMission(uavId = 0, missionId = 0, routeId = 1) {
    listSM[uavId] = createActor(machine).start();
    listSM[uavId].subscribe((state) => {
      logger.debug(`State machine uav=${uavId} state=${state.value} context=${JSON.stringify(state.context)}`);
      MissionController.updateMission({
        device: state.context.uavId,
        mission: state.context.missionId,
        state: state.value,
      });
    });
    listSM[uavId].send({ type: 'ChangeId', value: { uavId, missionId, routeId } });
    return true;
  }

  static get_status(id) {
    if (listSM.hasOwnProperty(id)) {
      return listSM[id].states;
    } else {
      logger.warn(`No state machine for UAV id=${id}`);
    }
    return null;
  }

  static load_mission(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].send({ type: 'loadMission' });
    } else {
      logger.warn(`No state machine for UAV id=${id}`);
    }
  }
  static command_mission(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].send({ type: 'commandMission' });
    } else {
      logger.warn(`No state machine for UAV id=${id}`);
    }
  }
  static UAVFinishMission(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].send({ type: 'downloadFilesUAV' });
    } else {
      logger.warn(`No state machine for UAV id=${id}`);
    }
    return true;
  }
  static DownloadFiles(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].send({ type: 'downloadFilesGCS' });
    } else {
      logger.warn(`No state machine for UAV id=${id}`);
    }
    return true;
  }
  static FinishMission(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].send({ type: 'FinishMission' });
    } else {
      logger.warn(`No state machine for UAV id=${id}`);
    }
  }
  static DeleteActor(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].stop();
      delete listSM[id];
      logger.info(`State machine deleted for UAV id=${id}`);
    } else {
      logger.warn(`No state machine for UAV id=${id}`);
    }
  }
}
