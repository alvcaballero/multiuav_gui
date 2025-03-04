import { devicesController } from './devices.js';
import { rosController } from './ros.js';
import { positionsController } from './positions.js';
import { planningController } from './planning.js';
export class websocketController {
  static async init() {
    const devices = await devicesController.getAllDevices();
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
    if (Object.values(positions).length) {
      currentsocket['positions'] = positions;
    }
    if (Object.values(camera).length) {
      currentsocket['camera'] = Object.values(camera);
    }
    return JSON.stringify(currentsocket);
  }

  static async updateserver() {
    const devices = await devicesController.getAllDevices();
    const server = await rosController.getServerStatus();

    let response = JSON.stringify({
      server: { rosState: server.state },
      devices: Object.values(devices),
    });
    return response;
  }
}
