import { eventsModel } from '../models/events.js';
import logger from '../common/logger.js';

export class eventsController {
  static async getAll(req, res) {
    logger.debug('Getting all events');
    const positions = await eventsModel.get({});
    res.json(positions);
  }

  static async getAllEvent() {
    return await eventsModel.get({});
  }

  static async addEvent(value) {
    const { type, eventTime, deviceId, missionId, positionId, attributes } = value;
    try {
      await eventsModel.addEvent({ type, eventTime, deviceId, missionId, positionId, attributes });
    } catch (err) {
      logger.warn(`addEvent failed: ${err.message}`);
    }
  }

  static async getByDeviceId(req, res) {
    position = {};
    res.json(position);
  }
}
