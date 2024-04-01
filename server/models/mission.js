import { DevicesController } from '../controllers/devices.js';
import { WebsocketManager } from '../WebsocketManager.js';
import { missionSMModel } from './missionSM.js';
import { ExtAppController } from '../controllers/ExtApp.js';
import { planningController } from '../controllers/planning.js';
import { FilesController } from '../controllers/files.js';
import { readYAML } from '../common/utils.js';

/* mission is object that have the current mission running and have the next object
 / id
 / initTime
 / FinishTime
 / status: init planning, running, finish, ,
 / uav: {uavId, status:  load, commmand , running , resumen cancel,Get Data, Download Data, InitTime,FinishTime}
 / mission: mission format yaml
*/
const Mission = {
  72510181: {
    id: 72510181,
    initTime: '2024-03-05T12:20:46',
    endTime: '2024-03-05T12:45:46',
    status: 'finish',
    mission: 'mission.yaml',
    results: [
      {
        latitude: 37.09241,
        longitude: -5.232,
        measures: [
          {
            name: 'minDistance',
            value: 25.34,
          },
        ],
      },
    ],
  },
};
/*// current mission // id , status (init, planing, doing, finish,time inti, time_end))
 */
const Routes = {
  1: {
    id: 1,
    status: 'finish',
    missionId: 72510181,
    deviceId: '14',
    initTime: '2024-03-05T12:20:46',
    endTime: '2024-03-05T12:45:46',
    result: {
      latitude: 37.09241,
      longitude: -5.232,
      measures: [
        {
          name: 'minDistance',
          value: 25.34,
        },
      ],
    },
  },
};

export class missionModel {
  static getmission(id) {
    console.log('Get mission' + id);
    if (id) {
      return Mission[id].mission;
    }
    return Mission[id].mission;
  }
  static getmissionValue(id) {
    if (id) {
      console.log('Get mission' + id);
      return Mission[id];
    }
    console.log('Get all mission');
    return Mission;
  }
  static getRoutes({ deviceId, missionId }) {
    if (deviceId) {
      return Object.values(Routes).filter((word) => word.deviceId == deviceId);
    }
    if (missionId) {
      return Object.values(Routes).filter((word) => word.missionId == missionId);
    }
    return Routes;
  }

  static sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
  static async decodeTask({ id, name, objetivo, locations, meteo }) {
    console.log('command-sendtask');

    let myTask = {};
    myTask.id = id;
    myTask.name = name ? name : 'automatic';
    myTask.locations = locations;
    myTask.case = planningController.getCaseTypes()[objetivo].case;
    myTask.meteo = meteo;

    let bases = planningController.getConfigBases();
    let devices = await DevicesController.getAllDevices();
    let param = planningController.getConfigParam(objetivo);
    let auxconfig = {};
    Object.keys(param['settings']).forEach((key1) => {
      auxconfig[key1] = param['settings'][key1].default;
    });

    let basesettings = planningController.getBasesSettings();
    myTask.devices = basesettings.map((setting, index) => {
      let config = { settings: {} };
      let myDevice = Object.values(devices).find((device) => device.id == setting.devices.id);
      for (const value of Object.keys(auxconfig)) {
        value !== 'base' ? (config.settings[value] = auxconfig[value]) : null;
      }
      config.settings.base = Object.values(bases[index]);
      config.settings.landing_mode = 2;
      config.id = myDevice.name;
      config.category = myDevice.category;
      return config;
    });
    console.log(myTask);
    console.log(myTask.devices);
    return myTask;
  }

  static async sendTask({ id, name, objetivo, locations, meteo }) {
    console.log('command-sendtask');

    let myTask = await this.decodeTask({ id, name, objetivo, locations, meteo });

    const isPlanning = false;
    if (isPlanning) {
      let mission = readYAML(`../config/mission/mission_1.yaml`);
      this.initMission(id, { ...mission, id: id });
      return { response: myTask, status: 'OK' };
    }
    planningController.PlanningRequest({ id, myTask });
    return { response: myTask, status: 'OK' };
  }

  static async initMission(missionId, mission) {
    const listUAV = mission.route.map((route) => DevicesController.getByName(route.uav).id);
    Mission[missionId] = {
      id: missionId,
      uav: listUAV,
      status: 'init',
      initTime: new Date(),
      mission: mission,
    };
    listUAV.forEach((uavId) => {
      missionSMModel.createActorMission(uavId, missionId);
    });
    ExtAppController.missionReqStart(missionId, mission);

    var ws = new WebsocketManager(null, '/api/socket');
    ws.broadcast(JSON.stringify({ mission: { ...mission, name: 'name' } }));
    return { response: mission, status: 'OK' };
  }
  static async UAVFinish(missionId, uavId) {
    let resultCode = 0;
    await ExtAppController.missionReqResult(missionId, resultCode);
    return true;
  }

  static async updateFiles(missionId, uavId) {
    const results = await FilesController.updateFiles(
      uavId,
      missionId,
      Mission[missionId].initTime
    );
    await this.sleep(5000);
    let code = 0;

    ExtAppController.missionReqMedia(missionId, { code, files: results.files, data: results.data });
    return true;
  }
  static async updateMission({ device, mission, state }) {
    return true;
  }
}
