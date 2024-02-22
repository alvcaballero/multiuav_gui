import { positionsModel } from '../models/positions.js';

export class positionsController {
  static async getAll(req, res) {
    console.log('controller get all');
    const positions = await positionsModel.getAll(req.query.deviceId);
    res.json(Object.values(positions));
  }

  static async getByDeviceId(req, res) {
    position = [];
    res.json(position);
  }
}
