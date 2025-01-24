import { eventsModel } from '../models/events.js';

export class eventsController {
  static async getAll(req, res) {
    console.log('controller get all');
    const positions = await eventsModel.get({});
    res.json(positions);
  }

  static async getAllEvent() {
    return await eventsModel.get({});
  }

  static async addEvent(value) {
    const { type, eventTime, deviceId, attributes } = value;
    const position = await eventsModel.addEvent({ type, eventTime, deviceId, attributes });
    return position;
  }

  static async getByDeviceId(req, res) {
    position = {};
    res.json(position);
  }
}
