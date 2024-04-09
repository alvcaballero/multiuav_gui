import { DevicesController } from './devices.js';
import { rosController } from './ros.js';
import { positionsController } from './positions.js';
//import { eventsController } from './events.js';
import { planningController } from './planning.js';
export class websocketController {
  static async init() {
    const devices = await DevicesController.getAllDevices();
    const positions = await positionsController.getLastPositions();
    const server = await rosController.getServerStatus();
    const planning = planningController.getDefaultPlanning();
    let response = JSON.stringify({
      positions: positions,
      server: { rosState: server.state },
      devices: Object.values(devices),
      markers: { bases: planning.markersbase, elements: planning.elements },
      planning: {
        id: planning.id,
        objetivo: planning.objetivo,
        loc: [],
        meteo: [],
        bases: planning.bases,
        settings: planning.settings,
      },
    });
    return response;
  }

  static async update() {
    let currentsocket = {};
    const positions = await positionsController.getLastPositions();
    const camera = await positionsController.getCamera();
    //const currentevent = await eventsController.getAllEvent();
    if (Object.values(positions).length) {
      currentsocket['positions'] = positions;
    }
    if (Object.values(camera).length) {
      currentsocket['camera'] = Object.values(camera);
    }

    //if (Object.values(getLastPositions).length) {
    //console.log(currentevent);
    //currentsocket['events'] = Object.values(currentevent);
    //eventsController.clearEvents({ eventId: Object.keys(currentevent) });
    //}
    //console.log('update');
    //console.log(currentsocket);
    return JSON.stringify(currentsocket);
  }

  static async updateserver() {
    const devices = await DevicesController.getAllDevices();
    const server = await rosController.getServerStatus();

    //console.log('devices');
    //console.log(devices);
    let response = JSON.stringify({
      server: { rosState: server.state },
      devices: Object.values(devices),
    });
    //console.log('updateserver');
    //console.log(response);
    return response;
  }
}
