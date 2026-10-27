import { eventBus, EVENTS } from '../../common/eventBus.js';
import { logger } from '../../common/logger.js';
import {
  RADIO_AP_HOST,
  RADIO_AP_COMMUNITY,
  RADIO_SNMP_VERSION,
  RADIO_POLL_INTERVAL_MS,
  RADIO_SNMP_TIMEOUT_MS,
} from '../../config/config.js';
import { LINK_LEVEL, findStation, hasRadio, levelOfSnr, linkOfStation } from './linkQuality.js';
import { fetchStations } from './ubiquitiSnmp.js';

// Polls an AP that stays silent (or a station missing from its table) this many times in a row before the link
// is called unknown / lost: one dropped SNMP datagram must not flash the chip.
const POLLS_TOLERATED = 3;

class DeviceLink {
  constructor() {
    this.link = null; // last values of the station (linkQuality.linkOfStation)
    this.level = LINK_LEVEL.UNKNOWN;
    this.seenAt = null; // ms, last poll that had the station in its table
  }
}

/**
 * Link quality between the GCS's radio and the one on board each drone. The GCS's Ubiquiti is an access point:
 * ONE query of its station table (SNMP, ubiquitiSnmp.js) gives the signal, noise and rates of every drone
 * associated to it, so nothing is asked of the drones. A device is matched to its row by `attributes.radioMac`
 * / `radioIp`; a device with neither is not tracked (`statusOf` → null, no chip).
 *
 * The ingestors are not involved: this is its own source, like telemetryWatchdog's clock. It reaches the client
 * as `device.link` (DevicesModel.getAll), and a change of level is re-broadcast at once as DEVICE_UPDATED.
 * Off while RADIO_AP_HOST is empty.
 */
class RadioLinkMonitor {
  constructor() {
    this._links = new Map(); // deviceId → DeviceLink
    this._timer = null;
    this._polling = false;
    this._apOkAt = null; // ms, last poll the AP answered
    this._apDown = false; // for logging the transitions only
    this._getDevices = async () => [];
    this._addEvent = () => {};
    this._fetch = () =>
      fetchStations({
        host: RADIO_AP_HOST,
        community: RADIO_AP_COMMUNITY,
        version: RADIO_SNMP_VERSION,
        timeoutMs: RADIO_SNMP_TIMEOUT_MS,
      });
  }

  /**
   * Dependencies are injected (not imported) for the same reason as telemetryWatchdog: `models/devices.js` reads
   * `statusOf`. `fetch` is replaceable for tests.
   */
  start({ getDevices, addEvent, fetch, host = RADIO_AP_HOST } = {}) {
    if (this._timer) return;
    if (!host) {
      logger.info('RadioLinkMonitor off (RADIO_AP_HOST is empty)');
      return;
    }
    this._getDevices = getDevices ?? this._getDevices;
    this._addEvent = addEvent ?? this._addEvent;
    if (fetch) this._fetch = fetch;
    this._timer = setInterval(() => this._tick(), RADIO_POLL_INTERVAL_MS);
    logger.info(
      `RadioLinkMonitor started (AP ${host}, SNMP v${RADIO_SNMP_VERSION}, every ${RADIO_POLL_INTERVAL_MS}ms)`
    );
  }

  stop() {
    if (this._timer) {
      clearInterval(this._timer);
      this._timer = null;
      logger.info('RadioLinkMonitor stopped');
    }
  }

  /**
   * @returns {null | {level: string, signal?: number, noise?: number, snr?: number, txRate?: number|null,
   *   rxRate?: number|null, txCapacity?: number|null, rxCapacity?: number|null, name?: string, ageMs: number|null}}
   *   null when the device has no radio set or nothing has been polled for it yet. `ageMs` is the time since the
   *   AP last listed it, computed here so the client clock does not matter.
   */
  statusOf(deviceId, now = Date.now()) {
    const state = this._links.get(deviceId);
    if (!state) return null;
    return {
      level: state.level,
      ...(state.link ?? {}),
      ageMs: state.seenAt === null ? null : now - state.seenAt,
    };
  }

  async _tick() {
    if (this._polling) return; // a slow AP must not pile polls up
    this._polling = true;
    try {
      const devices = await this._getDevices();
      let stations = null;
      try {
        stations = await this._fetch();
        if (this._apDown) logger.info('RadioLinkMonitor: the access point answers again');
        this._apDown = false;
      } catch (err) {
        if (!this._apDown) logger.warn(`RadioLinkMonitor: the access point does not answer: ${err.message}`);
        this._apDown = true;
      }
      this.check(devices, stations);
    } catch (err) {
      logger.error(`RadioLinkMonitor: poll failed: ${err.message}`);
    } finally {
      this._polling = false;
    }
  }

  /**
   * Applies one poll. Devices whose level changed are re-broadcast at once.
   * @param {object[]} devices
   * @param {object[]|null} stations the AP's table, null when the AP did not answer
   * @returns {object[]} the devices whose level changed
   */
  check(devices, stations, now = Date.now()) {
    if (stations) this._apOkAt = now;
    const lostAfterMs = POLLS_TOLERATED * RADIO_POLL_INTERVAL_MS;

    const watched = devices.filter((d) => hasRadio(d.attributes));
    const ids = new Set(watched.map((d) => d.id));
    for (const id of this._links.keys()) if (!ids.has(id)) this._links.delete(id);

    const changed = [];
    for (const device of watched) {
      let state = this._links.get(device.id);
      if (!state) {
        state = new DeviceLink();
        this._links.set(device.id, state);
      }
      const before = state.level;

      if (stations) {
        const station = findStation(stations, device.attributes);
        if (station) {
          state.link = linkOfStation(station);
          state.seenAt = now;
          state.level = levelOfSnr(state.link.snr, before);
        } else if (state.seenAt === null || now - state.seenAt > lostAfterMs) {
          state.level = LINK_LEVEL.LOST;
        }
      } else if (this._apOkAt === null || now - this._apOkAt > lostAfterMs) {
        state.level = LINK_LEVEL.UNKNOWN;
      }

      if (state.level !== before) {
        changed.push(device);
        this._report(device, before, state);
      }
    }

    if (changed.length) {
      eventBus.emitSafe(
        EVENTS.DEVICE_UPDATED,
        changed.map((device) => ({ ...device, link: this.statusOf(device.id, now) }))
      );
    }
    return changed;
  }

  // Only what the operator should know about: a link that is poor or gone, and its return. good ↔ fair and the
  // first reading are not worth an Event, and `unknown` is the AP's problem (already logged), not the drone's.
  // A drone that is simply off when the server boots goes unknown → lost: that is not news either.
  _report(device, before, state) {
    const { level } = state;
    if (before === LINK_LEVEL.UNKNOWN && level === LINK_LEVEL.LOST) return;
    let message = null;
    let type = 'warning';
    if (level === LINK_LEVEL.LOST) message = `${device.name}: radio link lost (not in the access point's table)`;
    else if (level === LINK_LEVEL.POOR) message = `${device.name}: radio link is poor (SNR ${state.link.snr} dB)`;
    else if (
      (before === LINK_LEVEL.LOST || before === LINK_LEVEL.POOR) &&
      (level === LINK_LEVEL.GOOD || level === LINK_LEVEL.FAIR)
    ) {
      message = `${device.name}: radio link is back (SNR ${state.link.snr} dB)`;
      type = 'info';
    }
    if (!message) return;
    logger.warn(`RadioLinkMonitor: ${message}`);
    this._addEvent({ type, deviceId: device.id, attributes: { action: 'RadioLink', level, message } });
  }
}

export const radioLinkMonitor = new RadioLinkMonitor();
