import { devicesController } from './devices.js';
import { rosController } from './ros.js';
import { positionsController } from './positions.js';
import { planningController } from './planning.js';
import { logger } from '../common/logger.js';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { WS_POSITIONS_INTERVAL_MS, WS_STATE_INTERVAL_MS } from '../config/config.js';
import { encodeCameraFrame } from '../subscribers/cameraStreamSubscriber.js';

let wsController = null;

export class websocketController {
  constructor(wsManager) {
    this.wsManager = wsManager;

    this.interval_update = setInterval(this.updateclient.bind(this), WS_POSITIONS_INTERVAL_MS);
    this.interval_server = setInterval(this.updateserver.bind(this), WS_STATE_INTERVAL_MS);
    // wellcome msg
    this.setupWelcomeMessage();
  }

  setupWelcomeMessage() {
    this.wsManager.onClientConnect = async (client) => {
      const msg = await this.WelcomeMessage();
      this.sendMessage(msg, client);

      // Camera frames push on arrival now (see CameraStreamSubscriber), not on
      // the polling interval — so a client connecting mid-session needs an
      // explicit unicast of whatever's cached, or it sees nothing until the
      // next ROS frame for that device.
      const camera = await positionsController.getCamera();
      Object.values(camera).forEach((payload) => {
        this.sendBinary(encodeCameraFrame(payload), client);
      });
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

  sendBinary(buffer, client = null) {
    if (client) {
      client.send(buffer);
    } else {
      this.wsManager.broadcast(buffer);
    }
  }

  /**
   * Snapshot periódico de telemetría (positions).
   *
   * Camera ya no pasa por acá: se empuja frame a frame por CameraStreamSubscriber
   * apenas ROS publica uno nuevo (ver positionsController.updateCamera).
   *
   * El scheduler ya NO toca el socket: emite eventos de dominio y el
   * WebSocketSubscriber es el único adaptador de salida (pipeline unificado).
   * El shaping del mensaje de salida vive en el subscriber (OUTBOUND_MAP).
   */
  async updateclient() {
    try {
      const positions = await positionsController.getLastPositions();

      // Solo emitir si hay datos (idéntico al guard original)
      if (Object.values(positions).length) {
        eventBus.emitSafe(EVENTS.POSITION_UPDATED, positions);
      }
    } catch (error) {
      logger.error('Error in updateclient', {
        error: error.message,
        stack: error.stack,
      });
    }
  }

  /**
   * Snapshot periódico de estado (server + devices) — mismo pipeline por eventos.
   */
  async updateserver() {
    try {
      const devices = await devicesController.getAllDevices();
      const server = await rosController.getServerStatus();

      eventBus.emitSafe(EVENTS.SERVER_UPDATED, { rosState: server.state });
      eventBus.emitSafe(EVENTS.DEVICE_UPDATED, devices);
    } catch (error) {
      logger.error('Error in updateserver', {
        error: error.message,
        stack: error.stack,
      });
    }
  }

  /**
   * Limpia los intervals y recursos del controller
   */
  destroy() {
    logger.info('websocketController cleanup');
    clearInterval(this.interval_update);
    clearInterval(this.interval_server);
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
        assignments: planning.assignments || [],
      },
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
