import { positionsController } from '../controllers/positions.js';
import sequelize from '../common/sequelize.js';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { de } from 'zod/v4/locales';

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

  static async addEvent({ type = 'no', eventTime, deviceId, missionId, positionId, attributes = {} }) {
    console.log('type:', type);
    console.log('eventTime:', eventTime);
    console.log('deviceId:', deviceId);
    console.log('missionId:', missionId);
    console.log('positionId:', positionId);
    console.log('attributes:', attributes);
    let device_id = deviceId || null;
    if (deviceId) {
      const deviceExists = await sequelize.models.Device.findByPk(deviceId);
      if (!deviceExists) {
        console.warn(`addEvent: deviceId ${deviceId} not found in DB, skipping event insert`);
        device_id = null;
      }
    }

    let eventPosition2 = [0, 0, 0];
    if (device_id) {
      let eventPosition = positionsController.getByDeviceId(device_id);
      if (eventPosition) {
        eventPosition2 = [eventPosition.latitude, eventPosition.longitude, eventPosition.altitude];
      }
    }
    let myEvent = await sequelize.models.Event.create({
      type: type,
      eventTime: eventTime || undefined,
      deviceId: device_id,
      positionId: eventPosition2,
      missionId: missionId || null,
      attributes: attributes,
    });

    // Emitir evento al EventBus para que los subscribers lo manejen
    eventBus.emitSafe(EVENTS.EVENT_CREATED, myEvent);
  }
}
