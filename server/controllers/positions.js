import { positionsModel } from '../models/positions/positions.js';
import { PositionHistoryModel } from '../models/positions/positionHistory.js';
import { positionBroadcastBatcher } from '../models/positions/positionBroadcastBatcher.js';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { logger } from '../common/logger.js';

export class positionsController {
  /**
   * GET /api/positions
   *
   * Ramas según query params:
   *  - Without any params, it returns a list of last known positions (CACHE)
   *  - deviceID without from/to, returns last known position of DeviceIDs
   *  - `from` + `to` (+ opcional `deviceId`) → HISTORIA desde la DB (array plano
   *    de filas PositionHistory ordenadas por fixTime).
   */
  static async getAll(req, res) {
    const { deviceId, from, to } = req.query;
    const hasRange = from !== undefined || to !== undefined;

    // Without any params → SEND ALL CACHED POSITIONS (RAM)
    if (deviceId === undefined && !hasRange) {
      const positions = await positionsModel.getAll();
      return res.json(Object.values(positions));
    }

    let deviceIds = null;
    if (deviceId !== undefined && deviceId !== null) {
      const rawIds = (Array.isArray(deviceId) ? deviceId : [deviceId]).flatMap((value) => String(value).split(','));
      deviceIds = rawIds.map((value) => Number(value.trim()));
      const invalidIds = rawIds.filter((value, i) => value.trim() === '' || isNaN(deviceIds[i]));
      if (invalidIds.length > 0) {
        return res.status(400).json({ error: `Invalid deviceId(s): ${invalidIds.join(', ')}` });
      }
    }

    // deviceID without from/to → SEND CACHED POSITION for deviceIds (RAM)
    if (!hasRange && deviceIds !== null) {
      logger.debug(`Getting cached positions for deviceId(s): ${deviceIds.join(', ')}`);
      const cached = await positionsModel.getByDeviceIds(deviceIds);
      return res.json(cached);
    }

    // from/to + opcional `deviceId`) → send Positions from DB

    // if from/to, return historical positions from the DB
    if (deviceIds?.length > 1) {
      return res.status(400).json({ error: 'Only one deviceId is allowed when querying history with from/to' });
    }

    // validate from/to
    if ((from !== undefined && isNaN(Date.parse(from))) || (to !== undefined && isNaN(Date.parse(to)))) {
      return res.status(400).json({ error: 'from and to must be valid ISO 8601 date-times' });
    }

    const parsedDeviceId = deviceIds === null ? undefined : deviceIds[0];
    const fromDate = new Date(from);
    const toDate = new Date(to);

    logger.debug(`Getting position history device=${parsedDeviceId ?? 'all'} from=${from} to=${to}`);
    const { rows, truncated } = await PositionHistoryModel.getHistory({
      deviceId: parsedDeviceId,
      from: fromDate,
      to: toDate,
    });
    if (truncated) {
      res.set('X-Result-Truncated', 'true');
      logger.warn(`Position history query truncated at limit (device=${parsedDeviceId ?? 'all'})`);
    }
    return res.json(rows);
  }

  static async getLastPositions() {
    const positions = await positionsModel.getAll();
    return Object.values(positions);
  }
  static getByDeviceId(deviceId) {
    return positionsModel.getByDeviceId(deviceId);
  }
  static async updatePosition(payload) {
    const position = await positionsModel.updatePosition(payload);
    if (payload?.deviceId !== undefined && payload?.latitude !== undefined) {
      // Raw, per-message signal — mission tracking (or any other interested module)
      // subscribes independently; this controller doesn't know who's listening.
      eventBus.emitSafe(EVENTS.POSITION_RECEIVED, payload);
    }
    // No se emite acá directo: se apila en el batcher, que agrupa los devices
    // que cambiaron y los manda juntos a WS_POSITIONS_INTERVAL_MS (2 Hz), en vez
    // de un mensaje por device — evita re-renders del cliente por cada cambio
    // individual cuando hay varios dispositivos activos a la vez. Se manda el
    // estado YA mergeado (positionsModel.updatePosition), no el payload crudo,
    // porque un mensaje puede traer solo un subconjunto de campos (ej: batería).
    if (position) {
      positionBroadcastBatcher.stage(position);
    }
  }
  static updateCamera(payload) {
    positionsModel.updateCamera(payload);
    if (payload?.deviceId !== undefined && payload?.camera !== undefined) {
      // Raw, per-message signal — the camera stream subscriber pushes it to
      // clients immediately; this controller doesn't know who's listening.
      eventBus.emitSafe(EVENTS.CAMERA_RECEIVED, payload);
    }
  }
  static async getCamera() {
    return await positionsModel.getCamera();
  }
}
