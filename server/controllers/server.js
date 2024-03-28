import { serverModel } from '../models/server.js';

export class serverController {
  static async server(req, res) {
    let response = await serverModel.Serverconfig();
    res.json(response);
  }
  static async getDateTime(req, res) {
    let response = await serverModel.DateTime();
    res.json(response);
  }
}
