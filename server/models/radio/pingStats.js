import { LINK_LEVEL } from './linkQuality.js';

// Pure rules of the ping-based link quality to a drone's onboard computer. No I/O.

// Pings in a row without answer after which the host is called lost; fewer than this, with none answered yet,
// is still unknown (the first ping after boot or after an address change).
export const LOST_AFTER_PINGS = 3;

// First approximation, like the SNR ones: to be tuned with flights. Loss is over the window (one drop in 20 = 5 %).
const GOOD = { lossPct: 5, avgMs: 50 };
const FAIR = { lossPct: 15, avgMs: 150 };

/** The round trip time out of the output of Linux `ping -c 1` (`time=3.08 ms`, `time<1 ms`), null if there is none. */
export function parsePingRtt(stdout) {
  const match = /time[=<]\s*([\d.]+)\s*ms/.exec(stdout ?? '');
  return match ? Number(match[1]) : null;
}

const round1 = (value) => Math.round(value * 10) / 10;

/**
 * @param {(number|null)[]} samples oldest → newest; a number is the RTT in ms, null a ping with no answer
 * @returns {null | {rttMs: number|null, avgMs: number|null, jitterMs: number|null, lossPct: number, samples: number}}
 *   `jitterMs` is the mean absolute difference between consecutive answered pings (null with fewer than two).
 */
export function summarize(samples) {
  if (!samples.length) return null;
  const answered = samples.filter((s) => s !== null);
  const jitter = answered.slice(1).map((rtt, i) => Math.abs(rtt - answered[i]));
  return {
    rttMs: samples.at(-1),
    avgMs: answered.length ? round1(answered.reduce((a, b) => a + b, 0) / answered.length) : null,
    jitterMs: jitter.length ? round1(jitter.reduce((a, b) => a + b, 0) / jitter.length) : null,
    lossPct: Math.round(((samples.length - answered.length) / samples.length) * 100),
    samples: samples.length,
  };
}

export function levelOfPing(samples) {
  const summary = summarize(samples);
  if (!summary) return LINK_LEVEL.UNKNOWN;
  const lastN = samples.slice(-LOST_AFTER_PINGS);
  if (lastN.length === LOST_AFTER_PINGS && lastN.every((s) => s === null)) return LINK_LEVEL.LOST;
  if (summary.avgMs === null) return LINK_LEVEL.UNKNOWN;
  if (summary.lossPct <= GOOD.lossPct && summary.avgMs < GOOD.avgMs) return LINK_LEVEL.GOOD;
  if (summary.lossPct <= FAIR.lossPct && summary.avgMs < FAIR.avgMs) return LINK_LEVEL.FAIR;
  return LINK_LEVEL.POOR;
}
