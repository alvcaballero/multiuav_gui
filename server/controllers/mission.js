//import { missionModel } from '../models/mission.js';
import { missionModel } from '../models/mission-sql.js';
import { ExtApp } from '../models/ExtApp.js';
export class missionController {
  static async getmission(req, res) {
    console.log('get missions');
    const response = await missionModel.getmission();
    res.json(response);
  }
  static async sendTask(req, res) {
    let response = await missionModel.sendTask(req.body);
    res.json(response);
  }
  static async setMission(req, res) {
    let response = await missionModel.setMission(req.body);
    res.json(response);
  }

  static async updateFiles(req, res) {
    console.log('updates Files');
    let response = await missionModel.updateFiles(req.params);
    res.json(response);
  }

  static async showFiles(req, res) {
    console.log('show files');
    let response = await missionModel.showFiles(req.params);
    res.json(response);
  }

  static async listFiles(req, res) {
    console.log('list files');
    let response = await missionModel.listFiles(req.params);
    res.json(response);
  }

  static async test(req, res) {
    console.log('list files');
    let response = await ExtApp.UpdateToken();
    res.json(response);
  }
  static initMission(mission_id, data) {
    missionModel.initMission(mission_id, data);
  }
  static finishMission(missionId, deviceId) {
    return missionModel.UAVFinish(missionId, deviceId);
  }
  static getMissionRoute(missionId) {
    return missionModel.getmissionValue(missionId);
  }
  static getCurrentMission(mission_id) {
    return missionModel.getmission(mission_id);
  }
  static updateFiles(missionId, deviceId) {
    return missionModel.updateFiles(missionId, deviceId);
  }
  static updateMission({ device, mission, state }) {
    missionModel.updateMission({ device, mission, state });
    return true;
  }
}
