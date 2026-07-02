import { eventsModel } from '../models/events.js';
import logger from '../common/logger.js';

export class eventsController {
  static async getAll(req, res) {
    const { deviceId, type, from, to } = req.query;
    logger.debug(`Getting events deviceId=${deviceId} type=${type} from=${from} to=${to}`);
    const events = await eventsModel.get({ deviceId, type, from, to });
    res.json(events);
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
