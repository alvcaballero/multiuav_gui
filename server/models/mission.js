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
 / results:  [ reoutes results]
 */
const Mission = readJSON(missionsData);
/*// current mission // id , status (init, planing, doing, finish,time inti, time_end))
 */

const Routes = readJSON(routesData);
export const MISSION_STATUS = Object.freeze({
  INIT: 'init',
  PLANNING: 'planning',
  RUNNING: 'running',
  COMPLETED: 'finish', //uav finish but no complete, need to download files
  END: 'done', //uav finish and complete
  CANCELLED: 'cancelled',
  ERROR: 'error',
});
export const ROUTE_STATUS = Object.freeze({
  INIT: 1,
  LOADED: 2,
  COMMANDED: 3,
  RUNNING: 4,
  COMPLETED: 5, //uav finish but no complete, need to download files
  END: 6, //uav finish and complete
  CANCELLED: 7,
  ERROR: 8,
});

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
    return Object.values(Routes);
  }

  static async createMission(payload) {
    Mission[payload.id] = payload;
    writeJSON(missionsData, Mission);
  }
  static async createRoute(payload) {
    let id = Math.max(0, ...Object.keys(Routes).map((value) => Number(value))) + 1;
    Routes[id] = { ...payload, id };
    writeJSON(routesData, Routes);
    return Routes[id];
  }
  static async editMission(payload) {
    Mission[payload.id] = { ...Mission[payload.id], ...payload };
    writeJSON(missionsData, Mission);
  }
  static async editRoute(payload) {
    let myRoute = { ...Routes[payload.id], ...payload };
    Routes[myRoute.id] = myRoute;
    writeJSON(routesData, Routes);

    if (myRoute.status == ROUTE_STATUS.COMPLETED && myRoute.hasOwnProperty('missionId')) {
      let listRoutes = await this.getRoutes({ missionId: myRoute.missionId });
      if (listRoutes.every((route) => route.status == ROUTE_STATUS.COMPLETED)) {
        this.editMission({ id: myRoute.missionId, status: MISSION_STATUS.COMPLETED, endTime: new Date() });
      }
    }
    if (myRoute.status == ROUTE_STATUS.END && myRoute.hasOwnProperty('missionId')) {
      console.log('route end');
      let listRoutes = await this.getRoutes({ missionId: myRoute.missionId });
      console.log(listRoutes);
      console.log(listRoutes.every((route) => route.status == ROUTE_STATUS.END));
      if (listRoutes.every((route) => route.status == ROUTE_STATUS.END)) {
        console.log('all routes end');
        const results = listRoutes.map((route) => route.result);
        this.editMission({ id: myRoute.missionId, results, status: MISSION_STATUS.END, endTime: new Date() });
        this.FinishProcessFiles(myRoute.missionId);
      }
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

    let baseSettings = planningController.getBasesSettings();
    myTask.devices = baseSettings.map((setting, index) => {
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
    console.log(myTask);
    console.log(myTask.locations);
    return myTask;
  }

  static async sendTask({ id, name = 'no name', objetivo, locations, meteo }) {
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
      status: MISSION_STATUS.INIT,
      initTime: new Date(),
      endTime: null,
      mission: mission,
      results: [],
    });
    listUAV.forEach((uavId) => {
      let myroute = this.createRoute({
        status: ROUTE_STATUS.INIT,
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
    this.editRoute({ id: routeId, status: ROUTE_STATUS.COMPLETED, endTime: new Date() });

    await ExtAppController.missionReqResult(missionId, resultCode);
    return true;
  }

  static async updateFiles(missionId, uavId) {
    let routeId = Object.values(Routes).find((item) => item.deviceId == uavId && item.missionId == missionId).id;
    const results = await FilesController.updateFiles(uavId, missionId, routeId, Mission[missionId].initTime);
    //await this.sleep(5000);
    //let code = 0;
    //await ExtAppController.missionReqMedia(missionId, { code, files: results.files, data: results.data });
    //this.editRoute({ id: routeId, results: results.data });
    return true;
  }
  static async UAVEnd(missionId, uavId) {
    console.log('===== UAVEnd whole mission =====');
    const routeId = Object.values(Routes).find((item) => item.deviceId == uavId && item.missionId == missionId).id;
    const listfiles = await FilesController.getFilesInfo({ routeId });
    console.log('listfiles');
    console.log(listfiles);
    const results = {};
    let attributes = {};
    for (const file of listfiles) {
      if (file.attributes && file.attributes.hasOwnProperty('measures') && file.attributes.measures.length > 0) {
        for (const measure of file.attributes.measures) {
          if (measure.name && measure.value) {
            if (!results.hasOwnProperty(measure.name)) {
              results[measure.name] = measure.value;
              attributes = file.attributes;
            } else if (results[measure.name] < measure.value) {
              results[measure.name] = measure.value;
              attributes = file.attributes;
            }
          }
        }
      }
    }
    console.log('results');
    console.log(attributes);
    console.log(results);
    this.editRoute({ id: routeId, status: ROUTE_STATUS.END, result: attributes, endTime: new Date() });
    return true;
  }

  static async FinishProcessFiles(missionId) {
    console.log('===== FinishProcessFiles whole mission =====');
    let code = 0;
    let result = { files: [], data: {} };
    let myfiles = await FilesController.getFilesInfo({ missionId });
    result.files = myfiles.map((file) => `${file.route}${file.name}`);
    console.log('missiion');
    console.log(this.getMissionValue(missionId));
    result.data = this.getMissionValue(missionId).results;
    ExtAppController.missionReqMedia(missionId, { code, files: result.files, data: result.data });
    return true;
  }
  static async updateMission({ device, mission, state }) {
    return true;
  }
}
