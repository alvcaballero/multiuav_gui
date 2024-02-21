import dotenv from 'dotenv';

import { DevicesModel } from '../models/devices.js';
import { WebsocketManager } from '../WebsocketManager.js';
import { missionSMModel } from './missionSM.js';
import { planningModel } from './planning.js';
import { ExtApp } from './ExtApp.js';
import { filesModel } from './files.js';
import { targetType } from 'ssh2-sftp-client/src/constants.js';

const Mission = {}; // current mission // id , status (init, planing, doing, finish,time inti, time_end))
const requestPlanning = {};

dotenv.config();

const planningServer = process.env.PLANNING_SERVER ?? 'http://127.0.0.1:8004/';

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
      return Mission[id].mission;
    }
    return Mission[id];
  }

  static sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  static async sendTask({ mission_id, objetivo, loc, meteo }) {
    console.log('command-sendtask');

    let myTask = {};
    myTask.id = mission_id;
    myTask.name = 'automatic';
    myTask.loc = loc;
    myTask.objetivo = objetivo;
    myTask.bases = planningModel.getBases();
    myTask.meteo = meteo;
    let devices = await DevicesModel.getAll();
    console.log(devices);
    let basesettings = planningModel.getBasesSettings();
    console.log(basesettings);
    myTask.settings = basesettings.map((setting) => {
      let mySetting = JSON.parse(JSON.stringify(setting));
      let myDevice = Object.values(devices).find((device) => device.id == setting.devices.deviceId);
      mySetting.devices.deviceId = myDevice.name;
      mySetting.devices.category = myDevice.category;
      return mySetting;
    });
    console.log(myTask.settings);

    console.log(myTask);
    const response1 = await fetch(`${planningServer}/mission_request`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(myTask),
    });
    if (response1.ok) {
      const response2 = await response1.json();
      console.log(response2);

      requestPlanning[mission_id] = {};
      requestPlanning[mission_id]['count'] = 0;
      requestPlanning[mission_id]['interval'] = setInterval(() => {
        console.log('response Planning' + mission_id);
        this.fetchPlanning(mission_id);
      }, 5000);
    } else {
      throw Error(await response.text());
    }
    return { response: myTask, status: 'OK' };
  }

  static async fetchPlanning(mission_id) {
    console.log('Fetch planning ' + mission_id);
    const response = await fetch(`${planningServer}/get_plan?IDs=${mission_id}`);
    if (response.ok) {
      const data = await response.json();
      console.log(data);
      console.log('response to check planning');
      if (Array.isArray(data.results) && data.results.length > 0) {
        if (data.results[0].hasOwnProperty('route')) {
          clearInterval(requestPlanning[mission_id]['interval']);
          requestPlanning[mission_id]['count'] = 10;
          this.initMission(mission_id, data.results[0]);
        }
      }
    } else {
      throw Error(await response.text());
    }
    requestPlanning[mission_id]['count'] = requestPlanning[mission_id]['count'] + 1;
    if (requestPlanning[mission_id]['count'] > 3) {
      clearInterval(requestPlanning[mission_id]['interval']);
    }
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
    const listMedia = await filesModel.updateFiles(uavId, missionId, initTime);
    await this.sleep(5000);
    let resultCode = 0;

    ExtApp.missionMedia(missionId, resultCode, listMedia);
    return true;
  }

  static planning({ mission_id, objectivo, loc, meteo }) {
    let uav = 'uav_15';

    let home = [37.134092, -6.472401, 50];
    let reqRoute = Object.values(loc);
    let mission = {
      version: '3',
      route: [{ name: 'datetime', uav: uav, wp: [] }],
      status: 'OK',
    };
    let response = { uav: uav, points: [], status: 'OK' };
    response.points.push(home);
    for (let i = 0; i < reqRoute.length; i = i + 1) {
      let wp_len = reqRoute[i].length;
      for (let j = 0; j < wp_len; j = j + 1) {
        if (j == 0) {
          response.points.push([reqRoute[i][j]['lat'], reqRoute[i][j]['lon'], 50]);
        }
        response.points.push([reqRoute[i][j]['lat'], reqRoute[i][j]['lon'], 30]);
        if (j == +wp_len + -1) {
          response.points.push([reqRoute[i][j]['lat'], reqRoute[i][j]['lon'], 50]);
        }
      }
    }

    response.points.push(home);
    mission.route[0]['wp'] = response.points.map((element) => {
      return { pos: element };
    });
    return mission;
  }
}
