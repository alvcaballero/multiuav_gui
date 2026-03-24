import { positionsModel } from '../models/positions.js';
import logger from '../common/logger.js';

export class positionsController {
  static async getAll(req, res) {
    logger.debug('Getting all positions');
    const positions = await positionsModel.getAll(req.query.deviceId);
    res.json(Object.values(positions));
  }

  static async getLastPositions(deviceId) {
    const positions = await positionsModel.getAll(deviceId);
    return Object.values(positions);
  }
  static getByDeviceId(deviceId) {
    return positionsModel.getByDeviceId(deviceId);
  }
  static updatePosition(payload) {
    positionsModel.updatePosition(payload);
  }
  static updateCamera(payload) {
    positionsModel.updateCamera(payload);
  }
  static async getCamera() {
    return await positionsModel.getCamera();
  }
}
