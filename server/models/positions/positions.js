import { eventsController } from '../../controllers/events.js';
import { logger } from '../../common/logger.js';
import { eventBus, EVENTS } from '../../common/eventBus.js';
import { applyPatch, createDefaultPosition } from './positionReducers.js';
const positions = {};
const camera = {};

export class positionsModel {
  static async getAll(query) {
    return positions;
  }

  // Synchronous on purpose: it only reads the in-memory cache. missionSignals and events.js call it
  // without `await`; while it was async they read a Promise, and the mission anomalies,
  // wp_reached and the event position never worked (test/test-mission-signals.js).
  static getByDeviceId(deviceId) {
    return positions[deviceId];
  }
  // Posiciones cacheadas de una lista de ids (para GET /positions?id=31&id=42).
  // Devuelve solo las que existen en caché, como array.
  static async getByDeviceIds(ids) {
    return ids.map((id) => positions[id]).filter((p) => p !== undefined);
  }
  static async getCamera() {
    return camera;
  }
  /**
   * Precarga la caché en RAM con filas del histórico (una por device), para que
   * el mapa no aparezca vacío al arrancar hasta que llegue telemetría nueva.
   * No pisa un device que ya tenga posición viva en caché.
   * @param {object[]} rows - filas planas de PositionHistory.
   * @returns {number} cantidad de devices hidratados.
   */
  static hydrate(rows) {
    let count = 0;
    for (const row of rows) {
      if (!row || row.deviceId === undefined || row.deviceId === null) continue;
      if (positions[row.deviceId] !== undefined) continue; // ya hay dato vivo
      positions[row.deviceId] = {
        deviceId: row.deviceId,
        latitude: row.latitude ?? undefined,
        longitude: row.longitude ?? undefined,
        altitude: row.altitude ?? undefined,
        course: row.course ?? 0.0,
        speed: row.speed ?? 0.0,
        accuracy: 0.0,
        deviceTime: row.deviceTime ?? row.fixTime ?? undefined,
        attributes: { ...(row.attributes || {}) },
      };
      count += 1;
    }
    logger.info(`positionsModel hydrated ${count} device(s) from history`);
    return count;
  }
  static updateCamera(payload) {
    camera[payload.deviceId] = payload;
  }
  static removePosition({ id }) {
    delete positions[id];
    logger.debug(`Position removed for device id=${id}`);
  }
  static async updatePosition(payload) {
    if (payload === null) {
      return null;
    }

    positions[payload.deviceId] ??= createDefaultPosition(payload.deviceId);
    const position = positions[payload.deviceId];
    for (const effect of applyPatch(position, payload)) {
      // Who closes the task is the mission's business (missionSMModel), not the position cache's.
      if (effect.missionStopped) eventBus.emitSafe(EVENTS.DEVICE_MISSION_STOPPED, effect.missionStopped);
      else eventsController.addEvent(effect);
    }
    return position;
  }
}
