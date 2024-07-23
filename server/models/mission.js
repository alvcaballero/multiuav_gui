import { DevicesController } from '../controllers/devices.js';
import { WebsocketManager } from '../WebsocketManager.js';
import { missionSMModel } from './missionSM.js';
import { ExtAppController } from '../controllers/ExtApp.js';
import { planningController } from '../controllers/planning.js';
import { FilesController } from '../controllers/files.js';
import { eventsController } from '../controllers/events.js';
import { readJSON, readYAML, writeJSON } from '../common/utils.js';
import { missionsData, routesData } from '../config/config.js';

/* mission is object that have the current mission running and have the next object
 / id
 / initTime
 / FinishTime
 / status: init planning, running, finish, ,
 / uav: {uavId, status:  load, commmand , running , resumen cancel,Get Data, Download Data, InitTime,FinishTime}
 / mission: mission format yaml
*/
const Mission = readJSON(missionsData);
/*// current mission // id , status (init, planing, doing, finish,time inti, time_end))
 */
const Routes = readJSON(routesData);

export class missionModel {
  static getMissionValue(id) {
    if (id) {
      console.log('Get mission' + id);
      return Mission[id];
    }
    console.log('Get all mission');
    return Object.values(Mission);
  }
  static async getRoutes({ deviceId, missionId }) {
    if (deviceId) {
      return Object.values(Routes).filter((word) => word.deviceId == deviceId);
    }
    if (missionId) {
      return Object.values(Routes).filter((word) => word.missionId == missionId);
    }
    return Routes;
  }

  static async createMission(payload) {
    Mission[payload.id] = payload;
    writeJSON(missionsData, Mission);
  }
  static async createRoute(payload) {
    let id = Math.max(...Object.keys(Routes).map((key) => Number(key))) + 1;
    Routes[id] = { ...payload, id };
    writeJSON(routesData, Routes);
    return Routes[id];
  }
  static async editMission(payload) {
    Mission[payload.id] = { ...Mission[payload.id], ...payload };
    writeJSON(missionsData, Mission);
  }
  static async editRoute(payload) {
    Routes[payload.id] = { ...Routes[payload.id], ...payload };
    writeJSON(routesData, Routes);
    if (payload.status == 'finish' && payload.hasOwnProperty('missionId')) {
      let listRoutes = Object.values(Routes).filter((item) => item.missionId == payload.missionId);
      if (listRoutes.every((route) => route.status == 'finish')) {
        this.editMission({ id: payload.missionId, status: 'finish', endTime: new Date() });
      }
    }
    if (payload.hasOwnProperty('result') && payload.hasOwnProperty('missionId')) {
      Mission[payload.missionId].results.push(payload.result);
      writeJSON(missionsData, Mission);
    }
  }

  static sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
  static async decodeTask({ id, name, objetivo, locations, meteo }) {
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
    console.log(`Task ${myTask.id} ${myTask.name} ${myTask.case} ${myTask.devices.map((item) => item.id).flat()}`);
    return myTask;
  }

  static async sendTask({ id, name, objetivo, locations, meteo }) {
    console.log('command-sendtask');

    let myTask = await this.decodeTask({ id, name, objetivo, locations, meteo });

    const isPlanning = false;
    if (isPlanning) {
      let mission = readYAML(`../config/mission/mission_11.yaml`);
      let mission1 = readYAML(`../config/mission/mission_complete_v3.yaml`);
      this.initMission(id, { ...mission1, id: id });
      return { response: myTask, status: 'OK' };
    }
    planningController.PlanningRequest({ id, myTask });

    eventsController.addEvent({
      type: 'info',
      eventTime: new Date(),
      deviceId: -1,
      attributes: { message: 'Ext APP send task' },
    });
    return { response: myTask, status: 'OK' };
  }

  static async initMission(missionId, mission) {
    const listUAV = [];
    for (const route of mission.route) {
      let findDevice = await DevicesController.getByName(route.uav);
      console.log(findDevice);
      listUAV.push(findDevice.id);
    }
    //const listUAV = mission.route.map((route) => DevicesController.getByName(route.uav).id);
    this.createMission({
      id: missionId,
      uav: listUAV,
      status: 'init',
      initTime: new Date(),
      endTime: null,
      mission: mission,
      results: [],
    });
    listUAV.forEach((uavId) => {
      let myroute = this.createRoute({
        status: 'init',
        missionId: missionId,
        deviceId: uavId,
        initTime: new Date(),
        endTime: null,
        result: {},
      });
      missionSMModel.createActorMission(uavId, missionId, myroute.id);
    });

    ExtAppController.missionReqStart(missionId, mission);

    var ws = new WebsocketManager(null, '/api/socket');
    ws.broadcast(JSON.stringify({ mission: { ...mission, name: 'name' } }));
    return { response: mission, status: 'OK' };
  }

  static async deviceFinishSyncFiles({ name, id }) {
    let mydevice = await DevicesController.getByName(name);
    console.log(mydevice);
    console.log('mydevice finish download files' + mydevice.id);
    missionSMModel.DownloadFiles(mydevice.id);
    return true;
  }

  static async deviceFinishMission({ name, id }) {
    console.log(name);
    let mydevice = await DevicesController.getByName(name);
    console.log(mydevice);
    console.log('mydevice in finish mission ' + mydevice.id);
    missionSMModel.UAVFinishMission(mydevice.id);
    return true;
  }
  static async UAVFinish(missionId, uavId) {
    let resultCode = 0;
    let routeId = Object.values(Routes).find((item) => item.deviceId == uavId && item.missionId == missionId).id;
    this.editRoute({ id: routeId, status: 'finish', endTime: new Date() });

    await ExtAppController.missionReqResult(missionId, resultCode);
    return true;
  }

  static async updateFiles(missionId, uavId) {
    let routeId = Object.values(Routes).find((item) => item.deviceId == uavId && item.missionId == missionId).id;
    const results = await FilesController.updateFiles(uavId, missionId, routeId, Mission[missionId].initTime);
    await this.sleep(5000);
    let code = 0;
    await ExtAppController.missionReqMedia(missionId, { code, files: results.files, data: results.data });
    this.editRoute({ id: routeId, results: results.data });

    return true;
  }
  static async FinishProcessFiles(missionId, deviceId, results) {
    let code = 0;
    ExtAppController.missionReqMedia(missionId, { code, files: results.files, data: results.data });
    return true;
  }
  static async updateMission({ device, mission, state }) {
    return true;
  }
}
