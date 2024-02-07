import { missionModel } from '../models/mission.js';

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
}
