import { devicesController } from './devices.js';
import { rosController } from './ros.js';
import { positionsController } from './positions.js';
import { planningController } from './planning.js';

let wsController = null;

export class websocketController {
  constructor(wsManager) {
    this.wsManager = wsManager;

    this.interval_update = setInterval(this.updateclient.bind(this), 2000);
    this.interval_server = setInterval(this.updateserver.bind(this), 10000);
    // wellcome msg
    this.setupWelcomeMessage();
  }

  async setupWelcomeMessage() {
    this.wsManager.onClientConnect = async (client) => {
      const msg = await this.WelcomeMessage();
      this.sendMessage(msg, client);
    };
  }

  sendMessage(msg, client = null) {
    const serialized = typeof msg === 'string' ? msg : JSON.stringify(msg);
    if (client) {
      client.send(serialized);
    } else {
      this.wsManager.broadcast(serialized);
    }
  }

  async updateclient() {
    const msg = await this.updateMessage();
    this.sendMessage(msg, null);
  }

  async updateserver() {
    const msg = await this.serverUpdateMessage();
    this.sendMessage(msg, null);
  }

  async WelcomeMessage() {
    const devices = await devicesController.getAllDevices();
    const positions = await positionsController.getLastPositions();
    const server = await rosController.getServerStatus();
    const planning = planningController.getDefaultPlanning();
    return {
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
    };
  }

  async updateMessage() {
    let currentsocket = {};
    const positions = await positionsController.getLastPositions();
    const camera = await positionsController.getCamera();
    if (Object.values(positions).length) {
      currentsocket['positions'] = positions;
    }
    if (Object.values(camera).length) {
      currentsocket['camera'] = Object.values(camera);
    }
    return currentsocket;
  }

  async serverUpdateMessage() {
    const devices = await devicesController.getAllDevices();
    const server = await rosController.getServerStatus();

    return {
      server: { rosState: server.state },
      devices: Object.values(devices),
    };
  }
}

export function initWebsocketController(wsManager) {
  wsController = new websocketController(wsManager);
  return wsController;
}

export function getWebsocketController() {
  return wsController;
}
