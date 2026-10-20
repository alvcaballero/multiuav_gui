import { randomUUID } from 'node:crypto';
import mqtt from 'mqtt';
import { devicesController } from '../../controllers/devices.js';
import { positionsController } from '../../controllers/positions.js';
import { categoryModel } from '../category.js';
import { decodeRosMsg } from '../ros/rosDecode.js';
import { logger } from '../../common/logger.js';
import { SUBSCRIPTIONS, parseTopic, requestTopic } from './mqttTopics.js';

// The drone's bridge has its own service timeout (5 s in the plan); this one is the GCS's
// last line of defence, so it must be longer or a slow-but-valid reply would be reported
// as a timeout. A call never stays pending past it.
const DEFAULT_TIMEOUT_MS = 8000;
const DEVICE_CACHE_TTL_MS = 30000;

// PX4 fills invalid fields with NaN and some JSON serializers emit it as a bare token,
// which JSON.parse rejects — dropping the whole message for one bad field. Retry with
// those tokens as null. Returns undefined when the payload is not JSON at all.
function parseJson(text) {
  try {
    return JSON.parse(text);
  } catch {
    try {
      return JSON.parse(text.replace(/-?\b(?:NaN|nan|Infinity|inf)\b/g, 'null'));
    } catch {
      return undefined;
    }
  }
}

/**
 * Third device transport: telemetry and ROS service calls over MQTT, mirror of
 * FlatbufferServer. Telemetry reuses decodeRosMsg (the drone publishes the same JSON
 * shape rosbridge does); service calls are request/response correlated by `id`.
 *
 * `connect` is injectable so tests run without a broker.
 */
export class MqttServer {
  constructor(url, { connect = mqtt.connect, timeoutMs = DEFAULT_TIMEOUT_MS, deps = {} } = {}) {
    this.timeoutMs = timeoutMs;
    this.devices = deps.devices ?? devicesController;
    this.positions = deps.positions ?? positionsController;
    this.categories = deps.categories ?? categoryModel;
    this.pending = new Map(); // id -> { resolve, timer, device, type }
    this.presence = new Map(); // device name -> 'online' | 'offline'
    this._devices = new Map(); // device name -> { device, at }

    this.client = connect(url, { clientId: 'muav-gcs', clean: true, reconnectPeriod: 2000 });
    this.client.on('connect', () => this._onConnect());
    this.client.on('message', (topic, payload) => this._onMessage(topic, payload));
    this.client.on('error', (err) => logger.error(`MqttServer error: ${err.message}`));
    this.client.on('close', () => this._failAllPending('broker connection closed'));
    logger.info(`MqttServer connecting to ${url}`);
  }

  // ── Public API ────────────────────────────────────────────────────────────

  /**
   * Calls a ROS service on the drone. Resolves with `{ state, msg }` once the drone really
   * answers (or the timeout / an error closes the call) — unlike robofleet, which reports
   * success as soon as the bytes are sent.
   */
  async sendCommand({ uav_id, type, attributes }) {
    const device = await this.devices.getDevice(uav_id);
    if (!device) return { state: 'error', msg: `device ${uav_id} not found` };

    const serviceDef = this.categories.getCategory(device.category)?.services?.[type];
    if (!serviceDef) return { state: 'warning', msg: `${type} to ${device.name} dont have this service` };

    // The name becomes a topic segment: a wildcard or separator in it would publish elsewhere.
    if (/[+#/\s]/.test(device.name)) {
      return { state: 'error', msg: `device name "${device.name}" is not a valid MQTT topic segment` };
    }

    if (!this.client.connected) return { state: 'error', msg: 'MQTT broker not connected' };

    // `attributes` arrive already encoded for the serviceType (models/msgEncode.js, before the
    // gateway): MQTT carries the same JSON shape rosbridge does.
    const request = attributes ?? {};

    const id = randomUUID();
    const envelope = JSON.stringify({ id, ts: Date.now(), request });

    return new Promise((resolve) => {
      const timer = setTimeout(() => {
        this.pending.delete(id);
        resolve({ state: 'error', msg: `${type} to ${device.name}: timeout, no device response` });
      }, this.timeoutMs);
      this.pending.set(id, { resolve, timer, device: device.name, type });

      this.client.publish(requestTopic(device.name, type), envelope, { qos: 1, retain: false }, (err) => {
        if (err) this._settle(id, { state: 'error', msg: `Failed to publish ${type}: ${err.message}` });
      });
    });
  }

  getPresence(deviceName) {
    return this.presence.get(deviceName) ?? 'unknown';
  }

  disconnect() {
    this._failAllPending('server shutting down');
    this.client.end();
  }

  // ── Private ───────────────────────────────────────────────────────────────

  _onConnect() {
    logger.info('MqttServer connected to broker');
    // clean session: the broker forgot our subscriptions, so re-subscribe on every connect.
    this.client.subscribe(SUBSCRIPTIONS, { qos: 1 }, (err) => {
      if (err) logger.error(`MqttServer subscribe failed: ${err.message}`);
    });
  }

  async _onMessage(topic, payload) {
    const parsed = parseTopic(topic);
    if (!parsed) return;

    if (parsed.kind === 'status') {
      const state = payload.toString().trim();
      this.presence.set(parsed.device, state);
      logger.info(`MqttServer: ${parsed.device} is ${state}`);
      return;
    }

    const body = parseJson(payload.toString());
    if (body === undefined) {
      logger.warn(`MqttServer: non-JSON payload on ${topic}`);
      return;
    }

    if (parsed.kind === 'response') return this._onResponse(parsed, body);
    return this._onTelemetry(parsed, body);
  }

  async _onTelemetry({ device: deviceName, slot }, msg) {
    const device = await this._getDevice(deviceName);
    if (!device) return;

    const cfg = this.categories.getCategory(device.category)?.subscribers?.[slot];
    if (!cfg) {
      logger.debug(`MqttServer: ${deviceName} has no subscriber slot "${slot}"`);
      return;
    }

    const decoded = decodeRosMsg({
      msg,
      deviceId: device.id,
      type: slot,
      msgType: cfg.messageType,
      params: cfg,
    });
    if (decoded) this.positions.updatePosition(decoded);
  }

  _onResponse({ device, name }, body) {
    const entry = this.pending.get(body?.id);
    if (!entry) {
      logger.debug(`MqttServer: response without pending call (id=${body?.id}) from ${device}/${name}`);
      return;
    }
    const label = `${entry.type} to ${entry.device}`;

    // envelope.success=false means the call itself failed (service unavailable, bridge
    // timeout, bad request). The service's own verdict lives inside `response`.
    if (body.success === false) {
      this._settle(body.id, { state: 'error', msg: `${label}: ${body.error || 'call failed'}` });
      return;
    }
    const r = body.response ?? {};
    const ok = Boolean(r.success || r.result);
    this._settle(body.id, { state: ok ? 'success' : 'error', msg: `${label} ${ok ? 'ok' : 'error'}` });
  }

  _settle(id, result) {
    const entry = this.pending.get(id);
    if (!entry) return;
    clearTimeout(entry.timer);
    this.pending.delete(id);
    entry.resolve(result);
  }

  _failAllPending(reason) {
    for (const id of [...this.pending.keys()]) {
      this._settle(id, { state: 'error', msg: reason });
    }
  }

  // Telemetry arrives at tens of Hz per drone: cache the name -> device lookup.
  async _getDevice(name) {
    const hit = this._devices.get(name);
    if (hit && Date.now() - hit.at < DEVICE_CACHE_TTL_MS) return hit.device;
    const device = await this.devices.getByName(name);
    if (!device) {
      logger.warn(`MqttServer: message from unknown device "${name}"`);
      return null;
    }
    this._devices.set(name, { device, at: Date.now() });
    return device;
  }
}
