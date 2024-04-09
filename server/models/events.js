import { positionsController } from '../controllers/positions.js';
import { WebsocketManager } from '../WebsocketManager.js';

const events = {}; // list all events
var eventsCount = 0;

export class eventsModel {
  static getall() {
    return events;
  }
  static addEvent(payload) {
    let eventPosition = positionsController.getByDeviceId(payload.deviceId);
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
    var ws = new WebsocketManager(null, '/api/socket');
    ws.broadcast(JSON.stringify({ events: [events[eventsCount]] }));
    eventsCount = eventsCount + 1;
  }

  static clearEvents(payload) {
    payload.eventId.forEach((element) => {
      delete events[element];
    });
  }
}
