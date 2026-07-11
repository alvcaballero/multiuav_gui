import { positionsModel } from '../models/positions.js';
import { missionWpTracking } from '../models/mission/missionWpTracking.js';
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
    if (payload?.deviceId !== undefined && payload?.latitude !== undefined) {
      missionWpTracking
        .checkProgress(payload.deviceId, payload)
        .catch((err) => logger.debug(`WpTracking error device=${payload.deviceId}: ${err.message}`));
    }
  }
  static updateCamera(payload) {
    positionsModel.updateCamera(payload);
  }
  static async getCamera() {
    return await positionsModel.getCamera();
  }
}
