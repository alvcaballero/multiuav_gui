import { eventBus, EVENTS } from '../../common/eventBus.js';
import { logger } from '../../common/logger.js';
import { TELEMETRY_CHECK_INTERVAL_MS, TELEMETRY_SLOT_TIMEOUT_MS } from '../../config/config.js';
import { categoryModel, subscribesCamera } from '../category.js';

/** Health of one subscriber slot. Only the problems are exposed (`statusOf`); OK is the default. */
export const SLOT_HEALTH = Object.freeze({
  OK: 'ok',
  STALE: 'stale', // it arrived, then stopped: sensor or node down
  MISSING: 'missing', // never arrived since the device showed up: almost always config (topic name, version suffix)
});

// Weight of the newest window in the rate average: with a 2 s tick, a 0.5 Hz topic gets 0 or 1
// message per window, and the raw count would jump between 0 and 1 Hz.
const RATE_SMOOTHING = 0.5;

// Periods of silence a slot with a declared `rate_hz` tolerates: two lost messages in a row on a
// SiK radio must not flip it to stale.
const RATE_PERIODS_TOLERATED = 3;

/**
 * Silence after which a slot is stale/missing. `timeout_s` wins; else three periods of the
 * declared `rate_hz` (never below the global default); else the global default.
 */
export function slotTimeoutMs(cfg) {
  const explicit = Number(cfg?.timeout_s);
  if (explicit > 0) return explicit * 1000;
  const rate = Number(cfg?.rate_hz);
  if (rate > 0) return Math.max(TELEMETRY_SLOT_TIMEOUT_MS, (RATE_PERIODS_TOLERATED * 1000) / rate);
  return TELEMETRY_SLOT_TIMEOUT_MS;
}

class DeviceTrack {
  constructor(now) {
    this.firstSeen = now;
    this.lastAny = now;
    this.lastSeen = new Map(); // slot → ms
    this.health = new Map(); // slot → SLOT_HEALTH, only the slots that are not OK
    this.counts = new Map(); // slot → messages since the last check
    this.rates = new Map(); // slot → Hz (smoothed)
    this.ratesAt = now;
  }

  updateRates(now) {
    const dt = (now - this.ratesAt) / 1000;
    if (dt <= 0) return;
    for (const slot of this.lastSeen.keys()) {
      const hz = (this.counts.get(slot) ?? 0) / dt;
      const prev = this.rates.get(slot);
      this.rates.set(slot, prev === undefined ? hz : prev + RATE_SMOOTHING * (hz - prev));
    }
    this.counts.clear();
    this.ratesAt = now;
  }
}

/**
 * Watchdog por slot: avisa cuando un device manda telemetría pero le falta un slot de los
 * que declara su categoría en devices_msg.yaml (llega `position` pero no `battery`).
 * DeviceHealthMonitor solo mira el device entero, así que con un slot vivo lo da ONLINE.
 *
 * Los ingestores (ros.js, MqttServer, FlatbufferServer) llaman a `record()` en cada mensaje,
 * junto a rawTap y antes del decode: es un `Map.set`, nada a 50 Hz. Un timer revisa cada
 * TELEMETRY_CHECK_INTERVAL_MS.
 *
 * - Activo por defecto en todo slot suscripto. Un slot que solo publica por evento
 *   (`vehicle_command_ack`, `threat`) se excluye con `watchdog: false` en su entrada, y uno
 *   lento se ajusta con `timeout_s`: es un param de la entrada, nunca un branch por categoría.
 *   Sin `timeout_s`, un slot que declara `rate_hz` espera tres períodos (`slotTimeoutMs`).
 * - Un device que no manda NADA no se evalúa (se descarta su estado): eso es OFFLINE, y lo
 *   cubre DeviceHealthMonitor; acá daría un warning por cada slot.
 * - Avisa en las TRANSICIONES (warning al caer, info al volver), no en cada tick.
 * - También mide la tasa (Hz) de cada slot: `snapshotOf` → GET /api/devices/:id/telemetry.
 *   Solo cuándo y cuántos, NUNCA el contenido: el último mensaje lo guarda rawTap.
 */
class TelemetryWatchdog {
  constructor() {
    this._devices = new Map(); // deviceId → DeviceTrack
    this._timer = null;
    this._getDevices = async () => [];
    this._addEvent = () => {};
  }

  /**
   * Las dependencias se inyectan (y no se importan) porque `models/devices.js` lee `statusOf`
   * para armar cada device: importarlo desde acá cerraría el ciclo.
   */
  start({ getDevices, addEvent }) {
    if (this._timer) return;
    this._getDevices = getDevices;
    this._addEvent = addEvent;
    this._timer = setInterval(() => this._tick(), TELEMETRY_CHECK_INTERVAL_MS);
    logger.info(
      `TelemetryWatchdog started (interval=${TELEMETRY_CHECK_INTERVAL_MS}ms, timeout=${TELEMETRY_SLOT_TIMEOUT_MS}ms)`
    );
  }

  stop() {
    if (this._timer) {
      clearInterval(this._timer);
      this._timer = null;
      logger.info('TelemetryWatchdog stopped');
    }
  }

  record(deviceId, slot, now = Date.now()) {
    if (deviceId == null || !slot) return;
    let track = this._devices.get(deviceId);
    if (!track) {
      track = new DeviceTrack(now);
      this._devices.set(deviceId, track);
    }
    track.lastAny = now;
    track.lastSeen.set(slot, now);
    track.counts.set(slot, (track.counts.get(slot) ?? 0) + 1);
  }

  /** @returns {Object<string, string>} slot → SLOT_HEALTH, only the slots with a problem ({} when all is fine) */
  statusOf(deviceId) {
    const track = this._devices.get(deviceId);
    return track ? Object.fromEntries(track.health) : {};
  }

  /**
   * Every slot seen or flagged for a device: `{ slot: { health, hz, ageMs } }`. `hz` is the
   * smoothed rate at the last check (null until one ran), `ageMs` the time since the last message
   * (null if it never arrived). Computed on the server so the client clock does not matter.
   * @returns {Object<string, {health: string, hz: number|null, ageMs: number|null}>}
   */
  snapshotOf(deviceId, now = Date.now()) {
    const track = this._devices.get(deviceId);
    if (!track) return {};
    const slots = new Set([...track.lastSeen.keys(), ...track.health.keys()]);
    return Object.fromEntries(
      [...slots].map((slot) => {
        const seen = track.lastSeen.get(slot);
        return [
          slot,
          {
            health: track.health.get(slot) ?? SLOT_HEALTH.OK,
            hz: track.rates.get(slot) ?? null,
            ageMs: seen === undefined ? null : now - seen,
          },
        ];
      })
    );
  }

  async _tick() {
    try {
      this.check(await this._getDevices());
    } catch (err) {
      logger.error(`TelemetryWatchdog: check failed: ${err.message}`);
    }
  }

  /**
   * Evaluates every device against the slots its category declares. Devices whose health
   * changed are re-broadcast at once, so the UI does not wait for the next periodic update.
   * @returns {object[]} the devices whose health changed
   */
  check(devices, now = Date.now()) {
    const known = new Set(devices.map((d) => d.id));
    for (const id of this._devices.keys()) if (!known.has(id)) this._devices.delete(id);

    const changed = devices.filter((device) => this._checkDevice(device, now));
    if (changed.length) {
      eventBus.emitSafe(
        EVENTS.DEVICE_UPDATED,
        changed.map((device) => ({ ...device, telemetry: this.statusOf(device.id) }))
      );
    }
    return changed;
  }

  _checkDevice(device, now) {
    const track = this._devices.get(device.id);
    if (!track) return false;
    if (now - track.lastAny > TELEMETRY_SLOT_TIMEOUT_MS) {
      // Silent as a whole: device-level problem. Dropping the track restarts the grace period
      // of every slot when it comes back.
      this._devices.delete(device.id);
      return track.health.size > 0;
    }
    track.updateRates(now);

    const subscribers = categoryModel.getCategory(device.category)?.subscribers ?? {};
    const watchCamera = subscribesCamera(subscribers, device.camera);
    let changed = false;
    const expected = new Set();

    for (const [slot, cfg] of Object.entries(subscribers)) {
      if (cfg?.watchdog === false || (slot === 'camera' && !watchCamera)) continue;
      expected.add(slot);

      const timeout = slotTimeoutMs(cfg);
      const seen = track.lastSeen.get(slot);
      let health = SLOT_HEALTH.OK;
      if (seen !== undefined && now - seen > timeout) health = SLOT_HEALTH.STALE;
      if (seen === undefined && now - track.firstSeen > timeout) health = SLOT_HEALTH.MISSING;

      const prev = track.health.get(slot) ?? SLOT_HEALTH.OK;
      if (health === prev) continue;
      if (health === SLOT_HEALTH.OK) track.health.delete(slot);
      else track.health.set(slot, health);
      this._report(device, slot, cfg, prev, health, timeout);
      changed = true;
    }

    // A slot removed from the category (runtime edit) stops counting as a problem.
    for (const slot of track.health.keys()) {
      if (!expected.has(slot)) {
        track.health.delete(slot);
        changed = true;
      }
    }
    return changed;
  }

  _report(device, slot, cfg, prev, health, timeout) {
    const seconds = timeout / 1000;
    const message = {
      [SLOT_HEALTH.STALE]: `${device.name}: no ${slot} data for more than ${seconds}s`,
      // MAVLink slots have no topic `name`: the message type is what identifies them there
      [SLOT_HEALTH.MISSING]: `${device.name}: ${slot} never arrived (${cfg?.name ?? cfg?.messageType ?? '?'})`,
      [SLOT_HEALTH.OK]: `${device.name}: ${slot} data is back`,
    }[health];
    logger.warn(`TelemetryWatchdog: ${message} (was ${prev})`);
    this._addEvent({
      type: health === SLOT_HEALTH.OK ? 'info' : 'warning',
      deviceId: device.id,
      attributes: { action: 'TelemetryWatchdog', slot, health, message },
    });
  }
}

export const telemetryWatchdog = new TelemetryWatchdog();
