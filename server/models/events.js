import { positionsController } from '../controllers/positions.js';
import sequelize, { Op } from '../common/sequelize.js';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { getDatetime } from '../common/utils.js';
import logger from '../common/logger.js';

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
  static async get({ id, missionId, deviceId, type, from, to }) {
    if (id) {
      return await sequelize.models.Event.findByPk(id);
    }

    const where = {};
    if (deviceId) where.deviceId = deviceId;
    if (missionId) where.missionId = missionId;
    if (type) where.type = type;
    if (from || to) {
      where.eventTime = {};
      if (from) where.eventTime[Op.gte] = new Date(from);
      if (to) where.eventTime[Op.lte] = new Date(to);
    }

    return await sequelize.models.Event.findAll({ where, order: [['eventTime', 'DESC']] });
  }

  static async addEvent({ type = 'no', eventTime, deviceId, missionId, positionId, attributes = {} }) {
    logger.debug(`addEvent: type=${type} eventTime=${eventTime} deviceId=${deviceId} missionId=${missionId} attributes=${JSON.stringify(attributes)}`);
    let device_id = deviceId || null;
    if (deviceId) {
      const deviceExists = await sequelize.models.Device.findByPk(deviceId);
      if (!deviceExists) {
        logger.warn(`addEvent: deviceId ${deviceId} not found in DB, skipping event insert`);
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
      eventTime: eventTime || getDatetime(),
      deviceId: device_id,
      positionId: eventPosition2,
      missionId: missionId || null,
      attributes: attributes,
    });

    // Emitir evento al EventBus para que los subscribers lo manejen
    eventBus.emitSafe(EVENTS.EVENT_CREATED, myEvent);
  }
}
