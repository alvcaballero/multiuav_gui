import { execFile } from 'node:child_process';
import { eventBus, EVENTS } from '../../common/eventBus.js';
import { logger } from '../../common/logger.js';
import { PING_ENABLE, PING_INTERVAL_MS, PING_WINDOW } from '../../config/config.js';
import { LINK_LEVEL } from './linkQuality.js';
import { levelOfPing, parsePingRtt, summarize } from './pingStats.js';

/**
 * One ICMP echo with Linux iputils `ping`. Resolves the RTT in ms, or null when nothing answered. Rejects only
 * when `ping` itself cannot run (ENOENT): that is the server's problem, not the drone's.
 * `execFile`, no shell: the address goes in as an argument (and it is a validated IPv4 anyway).
 * @returns {Promise<number|null>}
 */
export function pingOnce(ip) {
  return new Promise((resolve, reject) => {
    execFile(
      'ping',
      ['-n', '-c', '1', '-W', '1', ip],
      { timeout: 3000, env: { ...process.env, LC_ALL: 'C' } },
      (err, stdout) => {
        if (err?.code === 'ENOENT') return reject(err);
        resolve(err ? null : parsePingRtt(stdout)); // exit 1 = no reply
      }
    );
  });
}

class HostTrack {
  constructor(ip) {
    this.ip = ip;
    this.samples = []; // RTT ms or null, oldest first, at most PING_WINDOW
    this.level = LINK_LEVEL.UNKNOWN;
    this.lastAt = null; // ms, last sample taken
    this.busy = false; // a ping in flight: a slow host must not pile them up
  }
}

/**
 * Round trip to the onboard computer of each device that sets `attributes.companionIp`: ping every
 * PING_INTERVAL_MS, over a window of PING_WINDOW samples (RTT, jitter, loss). It complements the radio's SNR
 * (radioLinkMonitor): that is the radio hop only, this is the real path to the computer, and it works with any
 * radio. Reaches the client as `device.latency` (DevicesModel.getAll); a level change is re-broadcast at once.
 * A device without `companionIp` is not tracked (`statusOf` → null).
 */
class HostPingMonitor {
  constructor() {
    this._tracks = new Map(); // deviceId → HostTrack
    this._timer = null;
    this._getDevices = async () => [];
    this._addEvent = () => {};
    this._ping = pingOnce;
  }

  /** Dependencies are injected for the same reason as telemetryWatchdog's. `ping` is replaceable for tests. */
  start({ getDevices, addEvent, ping } = {}) {
    if (this._timer) return;
    if (!PING_ENABLE) {
      logger.info('HostPingMonitor off (PING_ENABLE=false)');
      return;
    }
    this._getDevices = getDevices ?? this._getDevices;
    this._addEvent = addEvent ?? this._addEvent;
    if (ping) this._ping = ping;
    this._timer = setInterval(() => this._tick(), PING_INTERVAL_MS);
    logger.info(`HostPingMonitor started (every ${PING_INTERVAL_MS}ms, window ${PING_WINDOW})`);
  }

  stop() {
    if (this._timer) {
      clearInterval(this._timer);
      this._timer = null;
      logger.info('HostPingMonitor stopped');
    }
  }

  /**
   * @returns {null | {level: string, rttMs: number|null, avgMs: number|null, jitterMs: number|null,
   *   lossPct: number, samples: number, ageMs: number}} null without a `companionIp` or before the first ping.
   */
  statusOf(deviceId, now = Date.now()) {
    const track = this._tracks.get(deviceId);
    const summary = track && summarize(track.samples);
    if (!summary) return null;
    return { level: track.level, ...summary, ageMs: now - track.lastAt };
  }

  async _tick() {
    try {
      const devices = await this._getDevices();
      this.track(devices);
      await Promise.all(
        [...this._tracks.entries()].map(async ([id, track]) => {
          if (track.busy) return;
          track.busy = true;
          try {
            this.record(id, await this._ping(track.ip));
          } finally {
            track.busy = false;
          }
        })
      );
      this.evaluate(devices);
    } catch (err) {
      if (err?.code === 'ENOENT') {
        logger.error('HostPingMonitor: `ping` is not installed on the server host (iputils-ping): stopping');
        this.stop();
      } else {
        logger.error(`HostPingMonitor: tick failed: ${err.message}`);
      }
    }
  }

  /** Follows the devices that have an address (a changed address starts over, a removed one is forgotten). */
  track(devices) {
    const wanted = new Map(
      devices.filter((d) => d.attributes?.companionIp).map((d) => [d.id, d.attributes.companionIp])
    );
    for (const id of this._tracks.keys()) if (!wanted.has(id)) this._tracks.delete(id);
    for (const [id, ip] of wanted) if (this._tracks.get(id)?.ip !== ip) this._tracks.set(id, new HostTrack(ip));
  }

  record(deviceId, rttMs, now = Date.now()) {
    const track = this._tracks.get(deviceId);
    if (!track) return;
    track.samples.push(rttMs);
    if (track.samples.length > PING_WINDOW) track.samples.shift();
    track.lastAt = now;
  }

  /**
   * Level of every tracked device. Those that changed are re-broadcast at once.
   * @returns {object[]} the devices whose level changed
   */
  evaluate(devices, now = Date.now()) {
    const changed = [];
    for (const device of devices) {
      const track = this._tracks.get(device.id);
      if (!track) continue;
      const before = track.level;
      track.level = levelOfPing(track.samples);
      if (track.level === before) continue;
      changed.push(device);
      this._report(device, before, track);
    }
    if (changed.length) {
      eventBus.emitSafe(
        EVENTS.DEVICE_UPDATED,
        changed.map((device) => ({ ...device, latency: this.statusOf(device.id, now) }))
      );
    }
    return changed;
  }

  // Same policy as radioLinkMonitor: only poor / lost and the way back; nothing for good ↔ fair, for the first
  // reading, or for a computer that is simply off when the server boots (unknown → lost).
  _report(device, before, track) {
    const { level } = track;
    if (before === LINK_LEVEL.UNKNOWN && level === LINK_LEVEL.LOST) return;
    const summary = summarize(track.samples);
    let message = null;
    let type = 'warning';
    if (level === LINK_LEVEL.LOST) message = `${device.name}: onboard computer ${track.ip} does not answer ping`;
    else if (level === LINK_LEVEL.POOR) {
      message = `${device.name}: link to the onboard computer is poor (${summary.avgMs} ms, ${summary.lossPct}% loss)`;
    } else if (
      (before === LINK_LEVEL.LOST || before === LINK_LEVEL.POOR) &&
      (level === LINK_LEVEL.GOOD || level === LINK_LEVEL.FAIR)
    ) {
      message = `${device.name}: link to the onboard computer is back (${summary.avgMs} ms)`;
      type = 'info';
    }
    if (!message) return;
    logger.warn(`HostPingMonitor: ${message}`);
    this._addEvent({ type, deviceId: device.id, attributes: { action: 'PingLink', level, message } });
  }
}

export const hostPingMonitor = new HostPingMonitor();
