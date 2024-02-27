import { machine } from './deviceSM.js';
import { createActor } from 'xstate';

const listSM = {}; // lista de acots maquinas de estados por id de UAV

export class missionSMModel {
  static createActorMission(uavId = 0, missionId = 0) {
    listSM[uavId] = createActor(machine).start();
    listSM[uavId].subscribe((state) => {
      console.log('state machine' + state.value);
      console.log('Value:', state.context);
    });
    listSM[uavId].send({ type: 'ChangeId', value: { uavId, missionId } });
    return true;
  }

  static get_status(id) {
    if (listSM.hasOwnProperty(id)) {
      return listSM[id].states;
    } else {
      console.log('no exist estate machine for this UAV = ' + id);
    }
    return null;
  }

  static load_mission(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].send({ type: 'loadMission' });
    } else {
      console.log('no exist estate machine for this UAV = ' + id);
    }
  }
  static command_mission(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].send({ type: 'commandMission' });
    } else {
      console.log('no exist estate machine for this UAV = ' + id);
    }
  }
  static UAVFinishMission(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].send({ type: 'downloadFilesUAV' });
    } else {
      console.log('no exist estate machine for this UAV = ' + id);
    }
  }
  static DownloadFiles(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].send({ type: 'downloadFilesGCS' });
    } else {
      console.log('no exist estate machine for this UAV = ' + id);
    }
  }
  static FinishMission(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].send({ type: 'FinishMission' });
    } else {
      console.log('no exist estate machine for this UAV = ' + id);
    }
  }
  static DeleteActor(id) {
    if (listSM.hasOwnProperty(id)) {
      listSM[id].stop();
      delete listSM[id];
      console.log('delete State machine ');
    } else {
      console.log('no exist estate machine for this UAV = ' + id);
    }
  }
}
