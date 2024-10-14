import { positionsController } from '../controllers/positions.js';
import { WebsocketManager } from '../WebsocketManager.js';

const events = {}; // list all events
var eventsCount = 0;

export class eventsModel {
  static getall() {
    return events;
  }
  static addEvent({ type, eventTime, deviceId, attributes }) {
    let eventPosition = positionsController.getByDeviceId(deviceId);
    let eventPosition2 = [0, 0, 0];
    if (eventPosition) {
      eventPosition2 = [eventPosition.latitude, eventPosition.longitude, eventPosition.altitude];
    }

    events[eventsCount] = {
      id: eventsCount,
      type: type,
      eventTime: eventTime || new Date().toISOString(),
      deviceId: deviceId || -1,
      positionid: eventPosition2,
      attributes: attributes,
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
