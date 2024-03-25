import { DevicesModel } from '../models/devices.js';
import { WebsocketManager } from '../WebsocketManager.js';
import { missionSMModel } from './missionSM.js';
import { planningModel } from './planning.js';
import { ExtApp } from './ExtApp.js';
import { filesModel } from './files.js';
import { readYAML } from '../common/utils.js';
import { planningController } from '../controllers/planning.js';
import Sequelize from '../common/sequelize.js';
/* mission is object that have the current mission running and have the next object
 / id
 / initTime
 / FinishTime
 / status: init planning, running, finish, ,
 / uav: {uavId, status:  load, commmand , running , resumen cancel,Get Data, Download Data, InitTime,FinishTime}
 / mission: mission format yaml
*/
const Mission = {}; // current mission // id , status (init, planing, doing, finish,time inti, time_end))
const Routes = {}; // id, missionId,uav,status, init time, end time
// const  files// id , route ,name, uav, date, attributes

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

  static sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  static async sendTask({ id, name, objetivo, locations, meteo }) {
    console.log('command-sendtask');

    let myTask = {};
    myTask.id = id;
    myTask.name = name ? name : 'automatic';
    myTask.locations = locations;
    myTask.case = planningModel.getTypes()[objetivo].case;
    myTask.meteo = meteo;

    let bases = planningModel.getBases();
    let devices = await DevicesModel.getAll();
    let param = planningModel.getParam(objetivo);
    let auxconfig = {};
    Object.keys(param['settings']).forEach((key1) => {
      auxconfig[key1] = param['settings'][key1].default;
    });

    let basesettings = planningModel.getBasesSettings();
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

    const isPlanning = true;
    if (isPlanning) {
      let mission = readYAML(`../config/mission/mission_1.yaml`);
      this.initMission(id, { ...mission, id: id });
      return { response: myTask, status: 'OK' };
    }
    planningController.PlanningRequest({ id, myTask });
    return { response: myTask, status: 'OK' };
  }

  static async initMission(missionId, mission) {
    const listUAV = mission.route.map((route) => DevicesModel.getByName(route.uav).id);
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
    ExtApp.missionStart(missionId, mission);

    var ws = new WebsocketManager(null, '/api/socket');
    ws.broadcast(JSON.stringify({ mission: { ...mission, name: 'name' } }));
    return { response: mission, status: 'OK' };
  }
  static async UAVFinish(missionId, uavId) {
    let resultCode = 0;
    await ExtApp.missionResult(missionId, resultCode);
    return true;
  }

  static async updateFiles(missionId, uavId) {
    const results = await filesModel.updateFiles(uavId, missionId, Mission[missionId].initTime);
    await this.sleep(5000);
    let code = 0;

    ExtApp.missionMedia(missionId, { code, files: results.files, data: results.data });
    return true;
  }
}
