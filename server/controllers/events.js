import { eventsModel } from '../models/events.js';

export class eventsController {
  static async getAll(req, res) {
    console.log('controller get all');
    const positions = await eventsModel.getAll();
    res.json(positions);
  }

  static async getByDeviceId(req, res) {
    position = {}
    res.json(position);
  }

}
