import { positionsModel } from '../models/positions/positions.js';
import { PositionHistoryModel } from '../models/positions/positionHistory.js';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { logger } from '../common/logger.js';

export class positionsController {
  /**
   * GET /api/positions
   *
   * Ramas según query params:
   *  - `from` + `to` (+ opcional `deviceId`) → HISTORIA desde la DB (array plano
   *    de filas PositionHistory ordenadas por fixTime).
   *  - `deviceId` SIN from/to → 400 (deviceId requiere rango, per spec).
   *  - `id=31&id=42` (o sin params) → posiciones CACHEADAS en RAM (comportamiento
   *    original), filtradas por esos ids o todas.
   */
  static async getAll(req, res) {
    const { deviceId, from, to, id } = req.query;
    const hasRange = from !== undefined || to !== undefined;

    // --- Rama HISTORIA (DB) ---
    if (hasRange || deviceId !== undefined) {
      // deviceId exige from y to (per OpenAPI spec).
      if (deviceId !== undefined && (from === undefined || to === undefined)) {
        return res.status(400).json({ error: 'deviceId requires the from and to parameters' });
      }
      // from/to deben ser fechas ISO 8601 válidas.
      const fromDate = from !== undefined ? new Date(from) : undefined;
      const toDate = to !== undefined ? new Date(to) : undefined;
      if ((from !== undefined && isNaN(fromDate)) || (to !== undefined && isNaN(toDate))) {
        return res.status(400).json({ error: 'from and to must be valid ISO 8601 date-times' });
      }

      const parsedDeviceId = deviceId !== undefined ? Number(deviceId) : undefined;
      if (parsedDeviceId !== undefined && isNaN(parsedDeviceId)) {
        return res.status(400).json({ error: 'deviceId must be an integer' });
      }

      logger.debug(`Getting position history device=${deviceId ?? 'all'} from=${from} to=${to}`);
      const { rows, truncated } = await PositionHistoryModel.getHistory({
        deviceId: parsedDeviceId,
        from: fromDate,
        to: toDate,
      });
      if (truncated) {
        res.set('X-Result-Truncated', 'true');
        logger.warn(`Position history query truncated at limit (device=${deviceId ?? 'all'})`);
      }
      return res.json(rows);
    }

    // --- Rama CACHÉ (RAM) ---
    if (id !== undefined) {
      const ids = Array.isArray(id) ? id : [id];
      logger.debug(`Getting cached positions ids=${ids.join(',')}`);
      const cached = await positionsModel.getByDeviceIds(ids);
      return res.json(cached);
    }

    logger.debug('Getting all cached positions');
    const positions = await positionsModel.getAll();
    return res.json(Object.values(positions));
  }

  static async getLastPositions() {
    const positions = await positionsModel.getAll();
    return Object.values(positions);
  }
  static getByDeviceId(deviceId) {
    return positionsModel.getByDeviceId(deviceId);
  }
  static updatePosition(payload) {
    positionsModel.updatePosition(payload);
    if (payload?.deviceId !== undefined && payload?.latitude !== undefined) {
      // Raw, per-message signal — mission tracking (or any other interested module)
      // subscribes independently; this controller doesn't know who's listening.
      eventBus.emitSafe(EVENTS.POSITION_RECEIVED, payload);
    }
  }
  static updateCamera(payload) {
    positionsModel.updateCamera(payload);
  }
  static async getCamera() {
    return await positionsModel.getCamera();
  }
}
