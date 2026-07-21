import { eventBus, EVENTS } from '../../common/eventBus.js';
import { logger } from '../../common/logger.js';
import { WS_POSITIONS_INTERVAL_MS } from '../../config/config.js';

/**
 * Agrupa los cambios de posición de varios dispositivos en un solo broadcast
 * por tick, en vez de un mensaje WS por cada device que cambia.
 *
 * ROS puede reportar telemetría hasta 50 Hz por dispositivo; positionsController
 * .updatePosition llama a `stage()` en cada mensaje, que solo pisa la última
 * posición conocida de ese device en un buffer en RAM (sin acumular históricos:
 * dos cambios del mismo device antes del flush colapsan en uno solo). Un timer
 * fijo a WS_POSITIONS_INTERVAL_MS (2 Hz) vacía el buffer y emite todos los
 * devices pendientes juntos — el buffer hace de dead-band temporal por device Y
 * de agrupador entre devices, así el cliente re-renderiza una vez por lote en
 * vez de una vez por cada cambio individual.
 */
class PositionBroadcastBatcher {
  constructor() {
    this._pending = new Map();
    this._timer = null;
  }

  start() {
    if (this._timer) return;
    this._timer = setInterval(() => this._flush(), WS_POSITIONS_INTERVAL_MS);
    logger.info(`PositionBroadcastBatcher started (interval=${WS_POSITIONS_INTERVAL_MS}ms)`);
  }

  stop() {
    if (this._timer) {
      clearInterval(this._timer);
      this._timer = null;
      logger.info('PositionBroadcastBatcher stopped');
    }
  }

  /**
   * Marca un device como pendiente de broadcast. `position` es la referencia
   * viva del cache de positionsModel: para cuando el timer la lea ya va a
   * reflejar el último merge, así que no hace falta clonarla acá.
   */
  stage(position) {
    if (!position || position.deviceId === undefined) return;
    this._pending.set(position.deviceId, position);
  }

  _flush() {
    if (this._pending.size === 0) return;
    const positions = Array.from(this._pending.values());
    this._pending.clear();
    eventBus.emitSafe(EVENTS.POSITION_UPDATED, positions);
  }
}

export const positionBroadcastBatcher = new PositionBroadcastBatcher();
