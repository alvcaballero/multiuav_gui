// Pure rules of the radio link quality: what a station row of the access point means. No I/O.

/** How good the link between the GCS's access point and a drone's radio is. */
export const LINK_LEVEL = Object.freeze({
  GOOD: 'good',
  FAIR: 'fair',
  POOR: 'poor',
  LOST: 'lost', // the AP answers and the station is not in its table
  UNKNOWN: 'unknown', // the AP does not answer: nothing is known about the link
});

// SNR (signal - noise floor, dB) that starts each level. First approximation: to be tuned with flights.
const SNR_GOOD_DB = 25;
const SNR_FAIR_DB = 15;
// A level is kept until the SNR drops this far below its threshold: a link sitting on a boundary must not
// flip (and raise an Event) at every poll.
const HYSTERESIS_DB = 2;

const RANK = { [LINK_LEVEL.POOR]: 0, [LINK_LEVEL.FAIR]: 1, [LINK_LEVEL.GOOD]: 2 };
const THRESHOLD = { [LINK_LEVEL.GOOD]: SNR_GOOD_DB, [LINK_LEVEL.FAIR]: SNR_FAIR_DB, [LINK_LEVEL.POOR]: -Infinity };

const rawLevel = (snr) =>
  snr >= SNR_GOOD_DB ? LINK_LEVEL.GOOD : snr >= SNR_FAIR_DB ? LINK_LEVEL.FAIR : LINK_LEVEL.POOR;

/**
 * @param {number} snr dB
 * @param {string} [previous] the level it had; only good/fair/poor count (lost/unknown restart from the raw one)
 */
export function levelOfSnr(snr, previous) {
  const level = rawLevel(snr);
  if (previous in RANK && RANK[level] < RANK[previous] && snr >= THRESHOLD[previous] - HYSTERESIS_DB) return previous;
  return level;
}

const mbps = (value, divisor) => (Number.isFinite(value) ? Math.round((value / divisor) * 10) / 10 : null);

/**
 * The link of one device from its row in the AP's station table (see ubiquitiSnmp.js `parseStations`).
 * `level` is NOT decided here: it needs the previous one (hysteresis), so the monitor sets it.
 * @returns {{signal: number, noise: number, snr: number, txRate: number|null, rxRate: number|null,
 *            txCapacity: number|null, rxCapacity: number|null, name: string, mac: string}}
 */
export function linkOfStation(station) {
  return {
    signal: station.signal,
    noise: station.noise,
    snr: station.signal - station.noise,
    txRate: mbps(station.txRate, 1e6), // Mbps (the AP gives bit/s)
    rxRate: mbps(station.rxRate, 1e6),
    txCapacity: mbps(station.txCapacity, 1e3), // Mbps (the AP gives kbit/s)
    rxCapacity: mbps(station.rxCapacity, 1e3),
    name: station.name,
    mac: station.mac,
  };
}

/** The station of the table that a device's radio is, by MAC first (an address can change) and then by IP. */
export function findStation(stations, { radioMac, radioIp } = {}) {
  if (radioMac) return stations.find((s) => s.mac === radioMac) ?? null;
  if (radioIp) return stations.find((s) => s.ip === radioIp) ?? null;
  return null;
}

/** A device has a radio to watch when either of its identifiers is set. */
export const hasRadio = (attributes) => Boolean(attributes?.radioMac || attributes?.radioIp);
