const events = {};
import { positionsModel } from '../models/positions.js';
var eventsCount = 0;

export class eventsModel {
  static getall() {
    return events;
  }
  static addEvent(payload) {
    let eventPosition = positionsModel.getDevice(payload.deviceId);
    let eventPosition2 = [0, 0, 0];
    if (eventPosition) {
      eventPosition2 = [eventPosition.latitude, eventPosition.longitude, eventPosition.altitude];
    }

    events[eventsCount] = {
      id: eventsCount,
      type: payload.type,
      eventTime: payload.eventTime,
      deviceId: payload.deviceId,
      positionid: eventPosition2,
      attributes: payload.attributes,
    };
    eventsCount = eventsCount + 1;
  }
  static clearEvents(payload) {
    payload.eventId.forEach((element) => {
      delete events[element];
    });
  }
}
