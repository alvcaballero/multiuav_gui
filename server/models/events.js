import { positionsController } from '../controllers/positions.js';
import { WebsocketManager } from '../WebsocketManager.js';
import sequelize from '../common/sequelize.js';


/**
 * @typedef Event
 * @property {integer} id
 * @property {string} type
 * @property {integer} deviceId
 * @property {Array<number>} positionid
 * @property {string} attributes
 * @property {string} createdAt - date-time create by DB
 */

export class eventsModel {
  static getall({ id, missionId, deviceId, type, createdAt }) {
    if (deviceId) {
      return sequelize.models.events.findAll({ where: { deviceId: deviceId } });
    }
    if (id) {
      return sequelize.models.events.findByPk(id);
    }
    return sequelize.models.events.findAll();
  }
  static async addEvent({ type, eventTime, deviceId, attributes }) {

    let eventPosition = positionsController.getByDeviceId(deviceId);

    let eventPosition2 = [0, 0, 0];
    if (eventPosition) {
      eventPosition2 = [eventPosition.latitude, eventPosition.longitude, eventPosition.altitude];
    }
    let myEvent = await sequelize.models.events.create({
      type: type,
      eventTime: eventTime || null,
      deviceId: deviceId || -1,
      positionid: eventPosition2,
      missionId: missionId || -1,
      attributes: attributes,
    });
    var ws = new WebsocketManager(null, '/api/socket');
    ws.broadcast(JSON.stringify({ events: [myEvent] }));
  }

  static clearEvents(payload) {
    payload.eventId.forEach((element) => {
      delete events[element];
    });
  }
}
