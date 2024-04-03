import { DevicesController } from '../controllers/devices.js';
import { WebsocketManager } from '../WebsocketManager.js';
import { missionSMModel } from './missionSM.js';
import { ExtAppController } from '../controllers/ExtApp.js';
import { planningController } from '../controllers/planning.js';
import { FilesController } from '../controllers/files.js';
import { readYAML } from '../common/utils.js';
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

/* Route is  object  that have a 
 / id
 / missionId
 / deviceId
 / status // state machine status 
 / initTime
 / endTime
*/
const Routes = {};

export class missionModel {
  static getmission(id) {
    console.log('Get mission' + id);
    if (id) {
      return Mission[id].mission;
    }
    return Mission[id].mission;
  }
  static getmissionValue(id) {
    console.log('Get mission' + id);
    if (id) {
      return Mission[id];
    }
    return Mission[id];
  }
  static getRoutes() {
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
    const myMission = sequelize.models.Mission.create({
      name: listUAV.toString(),
      mission: JSON.stringify(mission),
    });
    console.log('creation of mission' + myMission.id);
    Mission[missionId] = {
      id: missionId,
      uav: listUAV,
      status: 'init',
      initTime: new Date(),
      mission: mission,
    };
    listUAV.forEach((uavId) => {
      let myRoute = sequelize.models.Route.create({
        missionId: myMission.id,
        deviceId: uavId,
        mission: JSON.stringify(mission),
      });
      console.log(myRoute);
      Route[`${missionId}-${uavId}`] = {
        id: `${missionId}-${uavId}`,
        missionId: missionId,
        deviceId: uavId,
        status: 'init',
        result: 'uno',
      };
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
    routeId = 1;
    const results = await FilesController.updateFiles(
      uavId,
      missionId,
      routeId,
      Mission[missionId].initTime
    );
    await this.sleep(5000);
    let code = 0;

    ExtAppController.missionReqMedia(missionId, { code, files: results.files, data: results.data });
    return true;
  }
  static async updateMission({ device, mission, state }) {
    if (Route.hasOwnProperty(`${mission}-${device}`)) {
      Route[`${mission}-${device}`]['status'] = state;
      await sequelize.models.Route.update(
        { status: state },
        {
          where: {
            missionId: mission,
            deviceId: device,
          },
        }
      );
    }
    //Mission[missionId]['finishTime'] = 'sadf'
    return true;
  }
}
