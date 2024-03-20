import { rosModel } from '../models/ros.js';

export class rosController {
  static async getTopics(req, res) {
    console.log('controller get all');
    let response = await rosModel.getTopics();
    res.json(response);
  }
  static async getListMaster(req, res) {
    console.log('controller get all');
    let response = await rosModel.getListMaster();
    res.json(response);
  }
  static async subscribeDevice(uavAdded) {
    await rosModel.subscribeDevice(uavAdded);
  }
  static async unsubscribeDevice(id) {
    console.log('unsuscribe controller');
    let response = await rosModel.unsubscribeDevice(id);
    return response;
  }
  static async callService(message) {
    let response = await rosModel.callService(message);
    return response;
  }
  static async decodeMissionMsg(message) {
    let response = await rosModel.decodeMissionMsg(message);
    return response;
  }
  static getServerStatus() {
    return rosModel.serverStatus();
  }
}
