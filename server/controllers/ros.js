import { rosModel } from '../models/ros.js';
import { RosEnable } from '../config/config.js';
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
    return RosEnable ? await rosModel.subscribeDevice(uavAdded) : null;
  }
  static async unsubscribeDevice(id) {
    console.log('unsuscribe controller');
    return RosEnable ? await rosModel.unsubscribeDevice(id) : null;
  }
  static async callService(message) {
    if (!RosEnable) return { state: 'error', message: 'ROS connection is disabled' };
    let response = await rosModel.callService(message);
    return response;
  }

  static getServerStatus() {
    return rosModel.serverStatus();
  }
}
