import { mapModel } from '../models/map.js';

export class mapController {
  static async getElevation(req, res) {
    console.log('controller get all');
    let locations = req.query.locations;
    let response = await mapModel.ApiElevation(JSON.parse(locations));
    res.json(response);
  }
  static async calcElevation(req, res) {
    console.log('controller get all');
    let response = await mapModel.calcElevation(req.body.routes);
    res.json(response);
  }
}
