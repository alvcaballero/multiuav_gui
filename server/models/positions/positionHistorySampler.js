import { positionsModel } from './positions.js';
import { PositionHistoryModel } from './positionHistory.js';
import { logger } from '../../common/logger.js';
import { approxDistanceMeters } from '../../common/geo.js';

/**
 * Sampler del histórico de posiciones.
 *
 * Lee el snapshot vivo de `positionsModel` (alimentado a ~50 Hz por ROS/FlatBuffer)
 * a un ritmo fijo (SAMPLE_INTERVAL_MS) y persiste una fila SOLO si hubo un cambio
 * significativo respecto a la última guardada (dead-band). Si no hubo cambio pero
 * pasó demasiado tiempo (HEARTBEAT_MS), guarda igual para dejar constancia de que
 * el dispositivo seguía vivo y quieto.
 *
 * Desacopla la frecuencia de ingesta de la de persistencia, igual que el broadcast
 * WebSocket desacopla ingesta de UI.
 */

// Ritmo de muestreo.
const SAMPLE_INTERVAL_MS = 1_000;
// Guardado forzado aunque no haya cambios (anti-hueco).
const HEARTBEAT_MS = 30_000;

// Umbrales del dead-band (ajustables sin refactor).
const POS_THRESHOLD_M = 0.2; // distancia horizontal
const ALT_THRESHOLD_M = 0.2;
const COURSE_THRESHOLD_DEG = 3;
const SPEED_THRESHOLD_MS = 0.3;

// Atributos cuyo cambio (por valor) fuerza un guardado.
const KEY_ATTRIBUTES = [
  'mission_state',
  'wp_reached',
  'uav_state',
  'landed_state',
  'alarm',
  'armState',
  'navState',
  'failsafe',
  'commandAck',
  'MIC_1',
  'MIC_2',
  'MIC_3',
  'Metano',
  'Alcohol',
  'CO',
];

class PositionHistorySampler {
  constructor() {
    this._timer = null;
    // Última fila persistida por dispositivo: { row, savedAt }.
    this._lastSaved = new Map();
  }

  start() {
    if (this._timer) return;
    this._timer = setInterval(() => this._tick(), SAMPLE_INTERVAL_MS);
    logger.info(`PositionHistorySampler started (interval=${SAMPLE_INTERVAL_MS}ms, heartbeat=${HEARTBEAT_MS}ms)`);
  }

  /**
   * Precarga la caché en RAM con la última posición conocida (desde el histórico)
   * de cada device registrado y activo. Se llama una vez al arrancar el server,
   * antes de start(), para que el mapa no aparezca vacío hasta la primera
   * telemetría. Best-effort: cualquier error se loguea y no frena el arranque.
   *
   * DevicesModel se importa de forma dinámica para no acoplar estáticamente el
   * dominio positions al de devices (el import corre una sola vez al arrancar).
   */
  async preloadCache() {
    try {
      const { DevicesModel } = await import('../devices.js');
      const devices = await DevicesModel.getAll(); // ya filtra deletedAt: null
      const ids = devices.map((d) => d.id);
      if (ids.length === 0) {
        logger.info('PositionHistorySampler preload: no active devices');
        return 0;
      }
      const rows = await PositionHistoryModel.getLatestPerDevice(ids);
      const count = positionsModel.hydrate(rows);
      // Sembrar _lastSaved para que el dead-band compare contra lo precargado y no
      // re-guarde la misma posición apenas arranque el sampler.
      const now = Date.now();
      for (const row of rows) {
        if (!row || row.deviceId == null) continue;
        this._lastSaved.set(String(row.deviceId), {
          row: {
            latitude: row.latitude,
            longitude: row.longitude,
            altitude: row.altitude,
            course: row.course,
            speed: row.speed,
            attributes: row.attributes || {},
          },
          savedAt: now,
        });
      }
      return count;
    } catch (err) {
      logger.error(`PositionHistorySampler preload error: ${err.message}`);
      return 0;
    }
  }

  stop() {
    if (this._timer) {
      clearInterval(this._timer);
      this._timer = null;
      logger.info('PositionHistorySampler stopped');
    }
  }

  async _tick() {
    try {
      const snapshot = await positionsModel.getAll();
      const now = Date.now();
      for (const deviceId of Object.keys(snapshot)) {
        const pos = snapshot[deviceId];
        if (!pos) continue;
        // Sin fix de posición todavía: nada que guardar.
        if (pos.latitude === undefined || pos.longitude === undefined) continue;
        if (this._shouldSave(deviceId, pos, now)) {
          await this._save(deviceId, pos, now);
        }
      }
    } catch (err) {
      logger.error(`PositionHistorySampler tick error: ${err.message}`);
    }
  }

  /**
   * Decide si la muestra actual merece persistirse (dead-band + heartbeat).
   */
  _shouldSave(deviceId, pos, now) {
    const last = this._lastSaved.get(deviceId);
    if (!last) return true; // primera muestra del dispositivo

    // Heartbeat: forzar guardado si pasó demasiado tiempo sin registrar.
    if (now - last.savedAt >= HEARTBEAT_MS) return true;

    const prev = last.row;

    // Posición horizontal.
    if (prev.latitude != null && prev.longitude != null && pos.latitude != null && pos.longitude != null) {
      const dist = approxDistanceMeters(prev.latitude, prev.longitude, pos.latitude, pos.longitude);
      if (dist > POS_THRESHOLD_M) return true;
    }

    // Altitud, rumbo, velocidad.
    if (this._deltaExceeds(prev.altitude, pos.altitude, ALT_THRESHOLD_M)) return true;
    if (this._deltaExceeds(prev.course, pos.course, COURSE_THRESHOLD_DEG)) return true;
    if (this._deltaExceeds(prev.speed, pos.speed, SPEED_THRESHOLD_MS)) return true;

    // Cambios en atributos clave (estado / sensores).
    const prevAttrs = prev.attributes || {};
    const curAttrs = pos.attributes || {};
    for (const key of KEY_ATTRIBUTES) {
      if (JSON.stringify(prevAttrs[key]) !== JSON.stringify(curAttrs[key])) return true;
    }

    return false;
  }

  _deltaExceeds(a, b, threshold) {
    if (a == null || b == null) return a !== b; // aparición/desaparición del dato
    return Math.abs(a - b) > threshold;
  }

  /**
   * Normaliza el deviceTime del snapshot (ISO string / Date / ausente / inválido)
   * a un Date, o null si no es parseable. No debe romper el guardado.
   */
  _parseDeviceTime(value) {
    if (!value) return null;
    const d = value instanceof Date ? value : new Date(value);
    return Number.isNaN(d.getTime()) ? null : d;
  }

  async _save(deviceId, pos, now) {
    const row = {
      deviceId: Number(deviceId),
      fixTime: new Date(),
      deviceTime: this._parseDeviceTime(pos.deviceTime),
      latitude: pos.latitude ?? null,
      longitude: pos.longitude ?? null,
      altitude: pos.altitude ?? null,
      course: pos.course ?? null,
      speed: pos.speed ?? null,
      // Clon superficial del snapshot de attributes en el momento del guardado.
      attributes: { ...(pos.attributes || {}) },
    };
    const ok = await PositionHistoryModel.record(row);
    if (ok) {
      // Guardamos una copia desacoplada para comparar contra el snapshot vivo,
      // que se sigue mutando en RAM.
      this._lastSaved.set(deviceId, {
        row: {
          latitude: row.latitude,
          longitude: row.longitude,
          altitude: row.altitude,
          course: row.course,
          speed: row.speed,
          attributes: row.attributes,
        },
        savedAt: now,
      });
    }
  }
}

export const positionHistorySampler = new PositionHistorySampler();
