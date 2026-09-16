import { eventBus, EVENTS } from '../common/eventBus.js';
import { logger } from '../common/logger.js';

// Wire format for a camera frame WS message — self-describing so the client
// can tell it apart from any other binary stream added later:
// [1 byte msgType][1 byte len(deviceId)][deviceId UTF-8][JPEG bytes]
export const MSG_TYPE_CAMERA_FRAME = 1;

/**
 * Encodes a decoded camera payload ({deviceId, camera: Buffer}) into the wire
 * format above. Pure function — reused by CameraStreamSubscriber (per-frame
 * push) and websocketController.setupWelcomeMessage (unicast on connect), so
 * the framing is defined in exactly one place.
 */
export function encodeCameraFrame({ deviceId, camera }) {
  const idBytes = Buffer.from(String(deviceId), 'utf8');
  const header = Buffer.from([MSG_TYPE_CAMERA_FRAME, idBytes.length]);
  return Buffer.concat([header, idBytes, camera]);
}

/**
 * Subscriber que escucha CAMERA_RECEIVED (evento crudo, por-mensaje, emitido
 * desde positionsController.updateCamera) y empuja cada frame a los clientes
 * como frame WS binario — sin pasar por el pipeline JSON de WebSocketSubscriber.
 *
 * A diferencia de WebSocketSubscriber (declarativo, 100% JSON), este subscriber
 * necesita un path de envío binario, por eso vive en su propia clase.
 */
export class CameraStreamSubscriber {
  constructor(wsController) {
    if (!wsController) {
      throw new Error('CameraStreamSubscriber requires a websocketController instance');
    }

    this.wsController = wsController;
    logger.info('CameraStreamSubscriber initializing');
    this.handler = eventBus.onSafe(EVENTS.CAMERA_RECEIVED, (payload) => this.forward(payload));
  }

  forward(payload) {
    try {
      this.wsController.sendBinary(encodeCameraFrame(payload));
    } catch (error) {
      logger.error('CameraStreamSubscriber forward error', {
        error: error.message,
        stack: error.stack,
      });
    }
  }

  cleanup() {
    logger.info('CameraStreamSubscriber cleanup');
    eventBus.removeListener(EVENTS.CAMERA_RECEIVED, this.handler);
  }
}
