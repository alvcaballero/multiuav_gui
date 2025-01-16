import { DevicesController } from '../controllers/devices.js';
import { positionsController } from '../controllers/positions.js';
import { WebsocketManager } from '../WebsocketManager.js';
import { missionSMModel } from './missionSM.js';
import { ExtAppController } from '../controllers/ExtApp.js';
import { planningController } from '../controllers/planning.js';
import { FilesController } from '../controllers/files.js';
import { eventsController } from '../controllers/events.js';
import { readJSON, readYAML, writeJSON, sleep } from '../common/utils.js';
import { missionsData, routesData } from '../config/config.js';
import sequelize from '../common/sequelize.js';

/* mission is object that have the current mission running and have the next object
 / id
 / initTime
 / FinishTime
 / status: init, planning, running, finish, cancel, fail,
 / uav: {uavId, status:  load, commmand , running , resumen cancel,Get Data, Download Data, InitTime,FinishTime}
 / mission: mission format yaml
*/
// de once at time and after  multiple like devices
// this have to init  before sent to plannig
const Mission = {}; // current mission // id , status (init, planing, doing, finish,time inti, time_end))

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
  INIT: 'init',
  LOADED: 'loaded',
  COMMANDED: 'commanded',
  RUNNING: 'running',
  COMPLETED: 'complete', //uav finish but no complete, need to download files
  END: 'end', //uav finish and complete
  CANCELLED: 'cancelled',
  ERROR: 'error',
});


export class missionModel {
  static async getMissionValue(id) {
    console.log('Get mission' + id);
    if (id) {
      console.log('Get mission' + id);
      return await sequelize.models.Mission.findOne({ where: { id: id } });
    }
    console.log('Get all mission');
    return await sequelize.models.Mission.findAll();
  }

  static async getRoutes({ id, deviceId, missionId }) {
    if (deviceId && missionId) {
      return await sequelize.models.Route.findOne({ where: { deviceId: deviceId, missionId: missionId } });
    }
    if (id) {
      return await sequelize.models.Route.findOne({ where: { id: id } });
    }
    if (missionId) {
      return await sequelize.models.Route.findAll({ where: { missionId: missionId } });
    }
    return await sequelize.models.Route.findAll();
  }

  static async createMission({
    id,
    uav = [],
    status = MISSION_STATUS.INIT,
    initTime = new Date(),
    endTime = null,
    task = {},
    mission = {},
    results = [],
  }) {
    const myMission = await sequelize.models.Mission.create({ id, uav, status, initTime, endTime, task, mission, results });
    return myMission;
  }

  static async createRoute(payload) {
    const myRoute = sequelize.models.Route.create({ ...payload });
    return myRoute;
  }

  static async editMission({ id, uav, status, initTime, endTime, task, mission, results }) {
    let myMission = await sequelize.models.Mission.findOne({ where: { id: id } });
    if (!myMission) {
      return null;
    }
    if (status) myMission.status = status;
    if (uav) myMission.uav = uav;
    if (initTime) myMission.initTime = initTime;
    if (endTime) myMission.endTime = endTime;
    if (task) myMission.task = task;
    if (mission) myMission.mission = mission;
    if (results) myMission.results = results;
    await myMission.save();
    return myMission;
  }

  static async editRoute({ id, deviceId, missionId, status, initTime, endTime, task, mission, results }) {

    let myRoute = null
    if (id) await sequelize.models.Route.findOne({ where: { id: id } });
    if (deviceId && missionId) await sequelize.models.Route.findOne({ where: { deviceId: deviceId, missionId: missionId } });
    if (!myRoute) {
      return null;
    }
    if (status) myRoute.status = status;
    if (initTime) myRoute.initTime = initTime;
    if (endTime) myRoute.endTime = endTime;
    if (task) myRoute.task = task;
    if (mission) myRoute.mission = mission;
    if (results) myRoute.results = results;
    await myRoute.save();

    if (status === ROUTE_STATUS.COMPLETED) this.validateMission(missionId);
    return myRoute;
  }

  static async validateMission(missionId) {
    let allFinish = await sequelize.models.Route.findAll({ where: { missionId: missionId } }).every((route) => route.status == ROUTE_STATUS.COMPLETED);
    if (allFinish) {
      let myMission = await sequelize.models.Mission.findOne({ where: { id: missionId } });
      myMission.status = MISSION_STATUS.COMPLETED;
      await myMission.save();
      this.FinishProcessFiles(missionId);
    }
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
    // get setting of the task
    let param = planningController.getConfigParam(objetivo);
    let auxconfig = {};
    Object.keys(param['settings']).forEach((key1) => {
      auxconfig[key1] = param['settings'][key1].default;
    });
    console.log('-----param-------');
    console.log(param['devices']);

    let baseSettings = planningController.getBasesSettings();
    let devicesSettings = [];
    for (const [index, setting] of baseSettings.entries()) {
      console.log('bases setting');
      console.log(setting);
      let config = { settings: {} };
      let myDevice = Object.values(devices).find((device) => device.id == setting.devices.id);
      for (const value of Object.keys(auxconfig)) {
        value !== 'base' ? (config.settings[value] = auxconfig[value]) : null;
      }
      // verify if the device is allow to do the mission
      if (param.devices && param.devices['category']) {
        const auvAllow = param.devices['category'].type.some((item) => item == myDevice.category);
        console.log(`device ${myDevice.name} auvAllow ${auvAllow}`);
        if (!auvAllow) {
          continue;
        }
      }
      // filter devices are online
      if (myDevice == null || myDevice.status == 'offline') {
        console.log(`device offline ${myDevice.name}`);
        continue;
      }
      // filter devices are free
      let getRoutes = await this.getRoutes({ deviceId: myDevice.id });
      if (
        getRoutes.some(
          (route) =>
            route.status == ROUTE_STATUS.RUNNING ||
            route.status == ROUTE_STATUS.COMMANDED ||
            route.status == ROUTE_STATUS.LOADED ||
            route.status == ROUTE_STATUS.INIT
        )
      ) {
        console.log(`device ${myDevice.name} is busy`);
        continue;
      }

      config.id = myDevice.name;
      config.category = myDevice.category;
      config.settings.base = Object.values(bases[index]);
      config.settings.landing_mode = 2;
      let uavData = await positionsController.getLastPositions(myDevice.id);
      console.log('uavData');
      console.log(uavData);
      if (uavData && uavData[0]?.attributes?.batteryLevel) {
        if (!Number.isNaN(Number.parseFloat(uavData[0].attributes.batteryLevel))) {
          console.log(`device ${myDevice.name} battery ${uavData[0].attributes.batteryLevel}`);
          config.settings.battery_level = uavData[0].attributes.batteryLevel / 100;
        }
      }
      devicesSettings.push(config);
    }
    myTask.devices = devicesSettings.filter((item) => item != null);
    console.log(`Task ${myTask.id} ${myTask.name} ${myTask.case} ${myTask.devices.map((item) => item.id).flat()}`);
    console.log(myTask);
    console.log(myTask.devices);
    console.log(myTask.locations);
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

    this.createMission({ id, task: myTask });

    eventsController.addEvent({
      type: 'info',
      eventTime: new Date(),
      deviceId: -1,
      attributes: { message: 'Ext APP send task' },
    });
    return { response: myTask, status: 'OK' };
  }

  static async initMission(missionId, mission) {
    if (mission == null || !mission.hasOwnProperty('route') || mission.route.length == 0) {
      this.editMission({ id: missionId, status: MISSION_STATUS.ERROR });
      console.log('Mission ' + missionId + ' cant planning');
      return false;
    }
    const listUAV = [];
    for (const route of mission.route) {
      let findDevice = await DevicesController.getByName(route.uav);
      console.log(findDevice);
      listUAV.push(findDevice.id);
    }
    //const listUAV = mission.route.map((route) => DevicesController.getByName(route.uav).id);
    this.editMission({
      id: missionId,
      uav: listUAV,
      status: MISSION_STATUS.PLANNING,
      mission: mission,
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

    eventsController.addEvent({
      type: 'info',
      deviceId: -1,
      attributes: { message: `Init mission ${missionId}` },
    });
    var ws = new WebsocketManager(null, '/api/socket');
    ws.broadcast(JSON.stringify({ mission: { ...mission, name: 'name' } }));
    return { response: mission, status: 'OK' };
  }


  static async deviceFinishSyncFiles({ name, id }) {
    let mydevice = await DevicesController.getByName(name);
    if (mydevice == null) {
      console.log('mydevice name ' + name + ' not found');
      return false;
    }

    console.log(mydevice);
    console.log('mydevice finish download files' + mydevice.id);

    eventsController.addEvent({
      type: 'info',
      deviceId: mydevice.id,
      attributes: { message: `Finish mission ${mydevice.name}` },
    });
    missionSMModel.DownloadFiles(mydevice.id);
    return true;
  }

  static async deviceFinishMission({ name, id }) {
    console.log(name);
    let mydevice = await DevicesController.getByName(name);
    if (mydevice == null) {
      console.log('mydevice name ' + name + ' not found');
      return false;
    }
    console.log('mydevice in finish mission ' + mydevice.id);
    eventsController.addEvent({
      type: 'info',
      deviceId: mydevice.id,
      attributes: { message: `Finish mission ${mydevice.name}` },
    });
    missionSMModel.UAVFinishMission(mydevice.id);
    return true;
  }

  static async UAVFinish(missionId, uavId) {
    let resultCode = 0;
    this.editRoute({ missionId: missionId, deviceId: uavId, status: ROUTE_STATUS.COMPLETED, endTime: new Date() });

    eventsController.addEvent({
      type: 'info',
      deviceId: uavId,
      attributes: { message: `Device end` },
    });

    await ExtAppController.missionReqResult(missionId, resultCode);
    return true;
  }

  static async updateFiles(missionId, uavId) {
    routeId = 1;
    const routeId = this.getRoutes({ deviceId: uavId, missionId: missionId }).id;
    const initTime = this.getMissionValue(missionId).initTime;
    const results = await FilesController.updateFiles(uavId, missionId, routeId, initTime);
    await sleep(5000);
    //ExtAppController.missionReqMedia(missionId, { code, files: results.files, data: results.data });
    return true;
  }

  static async UAVEnd(missionId, uavId) {
    console.log('===== UAVEnd whole mission =====');
    const routeId = this.getRoutes({ missionId, uavId }).id;
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
            } else if (Number(results[measure.name]) < Number(measure.value)) {
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
    eventsController.addEvent({
      type: 'info',
      deviceId: uavId,
      attributes: { message: `device end ` },
    });
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
    eventsController.addEvent({
      type: 'info',
      deviceId: -1,
      attributes: { message: `Finish mission ${missionId}` },
    });
    ExtAppController.missionReqMedia(missionId, { code, files: result.files, data: result.data });
    return true;
  }

  static async updateMission({ device, mission, state }) {
    // await sequelize.models.Route.update(
    //   { status: state },
    //   {
    //     where: {
    //       missionId: mission,
    //       deviceId: device,
    //     },
    //   }
    // );
    return true;
  }
  static CheckLastMissionRoute() {
    Object.values(Mission).forEach((mission) => {
      if (
        mission.status == MISSION_STATUS.RUNNING ||
        mission.status == MISSION_STATUS.PLANNING ||
        mission.status == MISSION_STATUS.INIT
      ) {
        let listRoutes = Object.values(Routes).filter(
          (route) =>
            route.missionId == mission.id &&
            (route.status == ROUTE_STATUS.RUNNING ||
              route.status == ROUTE_STATUS.COMMANDED ||
              route.status == ROUTE_STATUS.LOADED ||
              route.status == ROUTE_STATUS.INIT)
        );
        for (const route of listRoutes) {
          this.editRoute({ id: route.id, status: ROUTE_STATUS.ERROR });
        }
        this.editMission({ id: mission.id, status: MISSION_STATUS.ERROR });
      }
    });
  }
}
missionModel.CheckLastMissionRoute();
