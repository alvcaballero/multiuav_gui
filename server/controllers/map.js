import { mapModel } from '../models/map.js';
import { logger } from '../common/logger.js';

export class mapController {
  static async getElevation(req, res) {
    logger.debug('Getting elevation data');
    let locations = req.query.locations;
    let response = await mapModel.ApiElevation(JSON.parse(locations));
    res.json(response);
  }
  static async calcElevation(req, res) {
    logger.debug('Calculating elevation profile');
    let response = await mapModel.calcElevation(req.body.routes);
    res.json(response);
  }
}
