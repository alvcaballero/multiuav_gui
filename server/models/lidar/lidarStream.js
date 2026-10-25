import { logger } from '../../common/logger.js';
import { LIDAR_LEVELS, DEFAULT_LIDAR_LEVEL, parsePointCloud2, reducePoints } from './pointCloud.js';
import { encodeLidarFrame } from './lidarFrame.js';

// A device's point cloud on demand. The cloud weighs ~2 MB/s per drone, so nothing subscribes to it
// until a client asks (the viewer of the device it has selected) and the ROS topic is closed with the
// last viewer. One ROS subscription per device serves every client; each client gets the reduced
// version of the level it asked for (pointCloud.js › LIDAR_LEVELS).
//
// The ROS side, the device lookup and the socket are injected (`start`) so this module knows none of
// them: server.js wires it like telemetryWatchdog.

/** Why a subscription was refused (the `reason` of the LIDAR_STATUS event). */
export const LIDAR_REJECT = Object.freeze({
  INVALID: 'invalid',
  UNKNOWN_DEVICE: 'unknown_device',
  NOT_SUPPORTED: 'not_supported',
  TOO_MANY: 'too_many',
  ROS_UNAVAILABLE: 'ros_unavailable',
  DEVICE_CHANGED: 'device_changed',
});

// A client views one device at a time today; the cap only stops a client from fanning out.
const MAX_DEVICES_PER_CLIENT = 4;
// A frame closer than this to the previous one of its level is still on time: the bridge's throttle
// is a minimum gap, and it jitters.
const ON_TIME = 0.9;

class LidarStream {
  constructor() {
    this._deps = null;
    // deviceId (String) → { device, clients: Map<clientId, level>, lastSent: Map<level, ms>, throttleMs, broken }
    this._devices = new Map();
    this._byClient = new Map(); // clientId → Set<deviceId>
  }

  /**
   * @param {object} deps
   * @param {(id) => Promise<object|null>} deps.getDevice
   * @param {(device) => boolean} deps.supports — the device has the stream (capability + interface slot)
   * @param {(device, { throttleMs, onMessage }) => { state: string }} deps.open — opens the ROS topic
   * @param {(device) => void} deps.close
   * @param {(clientId) => boolean} deps.isConnected
   * @param {(clientId, buffer) => boolean} deps.send — false when the frame was dropped (client too slow)
   * @param {(status: object) => void} deps.emitStatus — `{ clientId, deviceId, ok, reason?, level? }`
   * @param {() => number} [deps.now]
   */
  start(deps) {
    this._deps = { now: Date.now, ...deps };
  }

  async subscribe(clientId, deviceId, level = DEFAULT_LIDAR_LEVEL) {
    const reply = (result) => {
      this._deps.emitStatus({ clientId, deviceId, ...result });
      return result;
    };
    if (deviceId == null || !LIDAR_LEVELS[level]) return reply({ ok: false, reason: LIDAR_REJECT.INVALID });

    const key = String(deviceId);
    const held = this._byClient.get(clientId);
    if (!held?.has(key) && (held?.size ?? 0) >= MAX_DEVICES_PER_CLIENT) {
      return reply({ ok: false, reason: LIDAR_REJECT.TOO_MANY });
    }

    const device = await this._deps.getDevice(deviceId);
    // The client may have left while the device was being looked up; registering it now would leak a viewer.
    if (!this._deps.isConnected(clientId)) return { ok: false, reason: LIDAR_REJECT.INVALID };
    if (!device) return reply({ ok: false, reason: LIDAR_REJECT.UNKNOWN_DEVICE });
    if (!this._deps.supports(device)) return reply({ ok: false, reason: LIDAR_REJECT.NOT_SUPPORTED });

    // From here to the end nothing awaits, so two clients asking at once cannot open the topic twice.
    let entry = this._devices.get(key);
    const isNew = !entry;
    if (isNew) {
      entry = { device, clients: new Map(), lastSent: new Map(), throttleMs: 0, broken: false };
      this._devices.set(key, entry);
    }
    entry.clients.set(clientId, level);
    if (!this._byClient.has(clientId)) this._byClient.set(clientId, new Set());
    this._byClient.get(clientId).add(key);

    if (!this._retune(key, entry, isNew)) {
      this._drop(key, clientId);
      return reply({ ok: false, reason: LIDAR_REJECT.ROS_UNAVAILABLE });
    }
    return reply({ ok: true, level });
  }

  unsubscribe(clientId, deviceId) {
    this._drop(String(deviceId), clientId);
  }

  /** The socket of a client closed: its viewers die with it. */
  removeClient(clientId) {
    for (const key of [...(this._byClient.get(clientId) ?? [])]) this._drop(key, clientId);
  }

  /** The device was edited or deleted: what was open for it is stale, so its viewers are told and let go. */
  deviceGone(deviceId) {
    const key = String(deviceId);
    const entry = this._devices.get(key);
    if (!entry) return;
    for (const clientId of [...entry.clients.keys()]) {
      this._deps.emitStatus({ clientId, deviceId, ok: false, reason: LIDAR_REJECT.DEVICE_CHANGED });
      this._drop(key, clientId);
    }
  }

  /** ROS came back (every subscription died with the connection): open again what is still being watched. */
  reopenAll() {
    for (const [key, entry] of this._devices) this._retune(key, entry, true);
  }

  /** deviceId → clients watching it, for tests and diagnostics. */
  viewers() {
    return new Map([...this._devices].map(([key, entry]) => [key, new Map(entry.clients)]));
  }

  // The ROS topic is throttled to the quickest level someone asked for. Returns false when it could not be opened.
  _retune(key, entry, force) {
    const throttleMs = Math.min(...[...entry.clients.values()].map((level) => LIDAR_LEVELS[level].intervalMs));
    if (!force && throttleMs === entry.throttleMs) return true;
    const result = this._deps.open(entry.device, {
      throttleMs,
      onMessage: (msg) => this._onMessage(key, msg),
    });
    if (result?.state === 'error') {
      logger.warn(`Lidar stream of ${entry.device.name} not opened: ${result.msg}`);
      return false;
    }
    entry.throttleMs = throttleMs;
    return true;
  }

  _drop(key, clientId) {
    const entry = this._devices.get(key);
    entry?.clients.delete(clientId);
    const held = this._byClient.get(clientId);
    held?.delete(key);
    if (held?.size === 0) this._byClient.delete(clientId);
    if (!entry) return;
    if (entry.clients.size === 0) {
      this._devices.delete(key);
      this._deps.close(entry.device);
    } else {
      this._retune(key, entry, false);
    }
  }

  _onMessage(key, msg) {
    const entry = this._devices.get(key);
    if (!entry) return;
    const now = this._deps.now();

    // Cheap test first: most of the arrivals are not due for any level, and parsing is the cost.
    const due = new Set();
    for (const level of new Set(entry.clients.values())) {
      const last = entry.lastSent.get(level) ?? 0;
      if (now - last >= LIDAR_LEVELS[level].intervalMs * ON_TIME) due.add(level);
    }
    if (due.size === 0) return;

    let cloud;
    try {
      cloud = parsePointCloud2(msg);
    } catch (error) {
      if (!entry.broken) logger.warn(`Lidar stream of ${entry.device.name}: ${error.message}`);
      entry.broken = true;
      return;
    }
    entry.broken = false;

    for (const level of due) {
      entry.lastSent.set(level, now);
      const frame = encodeLidarFrame({
        deviceId: entry.device.id,
        level,
        cloud: reducePoints(cloud, LIDAR_LEVELS[level].maxPoints),
      });
      for (const [clientId, wanted] of entry.clients) {
        if (wanted === level) this._deps.send(clientId, frame);
      }
    }
  }
}

export const lidarStream = new LidarStream();
export { LidarStream };
