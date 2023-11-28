import { commandsModel } from '../models/commands.js';

export class commandsController {
  static async getAvalaibleCommands(req, res) {
    let deviceid = req.query.deviceId;
    let response = await commandsModel.getCommandTypes(deviceid);
    res.json(response);
  }
  static async getSaveCommands(req, res) {
    let deviceid = req.query.deviceId;
    let response = await commandsModel.getSaveCommands(deviceid);
    res.json(response);
  }
  static async sendCommand(req, res) {
    let response = await commandsModel.sendCommand(req.body);
    res.json(response);
  }
  static async sendTask(req, res) {
    let response = await commandsModel.sendTask(req.body);
    res.json(response);
  }
}
