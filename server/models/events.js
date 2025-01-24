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
  static async get({ id, missionId, deviceId, type, createdAt }) {
    if (deviceId) {
      return await sequelize.models.Event.findAll({ where: { deviceId: deviceId } });
    }
    if (id) {
      return await sequelize.models.Event.findByPk(id);
    }
    return await sequelize.models.Event.findAll();
  }

  static async addEvent({ type="no", eventTime, deviceId, missionId, positionId, attributes={} }) {
    console.log('type:', type);
    console.log('eventTime:', eventTime);
    console.log('deviceId:', deviceId);
    console.log('missionId:', missionId);
    console.log('positionId:', positionId);
    console.log('attributes:', attributes);

    let eventPosition2 = [0, 0, 0];
    if (deviceId) {
      let eventPosition = positionsController.getByDeviceId(deviceId);
      if (eventPosition) {
        eventPosition2 = [eventPosition.latitude, eventPosition.longitude, eventPosition.altitude];
      }
    }
    let myEvent = await sequelize.models.Event.create({
      type: type,
      eventTime: eventTime || null,
      deviceId: deviceId || null,
      positionId: eventPosition2,
      missionId: missionId || null,
      attributes: attributes,
    });
    var ws = new WebsocketManager(null, '/api/socket');
    ws.broadcast(JSON.stringify({ events: [myEvent] }));
  }

}
