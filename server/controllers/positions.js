import { positionsModel } from '../models/positions.js';

export class positionsController {
  static async getAll(req, res) {
    console.log('controller get all');
    const positions = await positionsModel.getAll(req.query.deviceId);
    res.json(Object.values(positions));
  }

  static async getLastPositions() {
    const positions = await positionsModel.getAll();
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
