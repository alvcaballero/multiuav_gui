/**
 * Mission tracking signal sources — each produces an independent estimate
 * of the current waypoint or an anomaly flag.
 *
 * All signals run in parallel on every position update.
 * The caller combines them; no signal overrides another.
 *
 * Signal return shape:
 *   { wpEstimate: number|null, confidence: 'high'|'medium'|'low'|null, anomaly: string|null }
 *
 * anomaly values (non-exclusive):
 *   'DEVIATION'   — UAV moving away from target WP
 *   'RTH_SUSPECTED' — nav/flight state suggests return-to-home
 *   'ON_GROUND'   — UAV appears landed while route is active
 *   'DISARMED'    — UAV disarmed mid-mission
 */

import { positionsModel } from '../positions.js';
import logger from '../../common/logger.js';

const DEG_TO_RAD = Math.PI / 180;
function haversineMeters(lat1, lon1, lat2, lon2) {
  const R = 6371000;
  const dLat = (lat2 - lat1) * DEG_TO_RAD;
  const dLon = (lon2 - lon1) * DEG_TO_RAD;
  const a =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1 * DEG_TO_RAD) * Math.cos(lat2 * DEG_TO_RAD) * Math.sin(dLon / 2) ** 2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

// ---------------------------------------------------------------------------
// Signal 1 — UAV flight state (armed / in-air / nav mode)
// Template: works for PX4 (armState/navState/failsafe) and DJI (landed_state).
// Returns null estimate — only produces anomaly flags.
// ---------------------------------------------------------------------------
export function signalFlightState(deviceId) {
  const pos = positionsModel.getByDeviceId(deviceId);
  if (!pos) return { wpEstimate: null, confidence: null, anomaly: null };

  const attr = pos.attributes ?? {};
  const anomalies = [];

  // PX4: armState comes from VehicleStatus
  if (attr.armState === 'Disarmed') {
    anomalies.push('DISARMED');
  }

  // PX4: RTH/land nav states
  const rthStates = ['Auto return to launch mode', 'Auto Land', 'Descend mode', 'Termination mode'];
  if (attr.navState && rthStates.includes(attr.navState)) {
    anomalies.push('RTH_SUSPECTED');
  }

  // DJI / CATEC: landed_state
  const landedStr = String(attr.landed_state ?? '').toUpperCase();
  if (landedStr === 'ON GROUND' || landedStr === 'STOPED') {
    anomalies.push('ON_GROUND');
  }

  return {
    wpEstimate: null,
    confidence: null,
    anomaly: anomalies.length ? anomalies.join(',') : null,
  };
}

// ---------------------------------------------------------------------------
// Signal 2 — Autopilot feedback (wp_reached / mission_state from drone itself)
// Template: only available for CATEC state_machine and some DJI configs.
// Most reliable when present — confidence HIGH.
// ---------------------------------------------------------------------------
export function signalAutopilotFeedback(deviceId, totalWp) {
  const pos = positionsModel.getByDeviceId(deviceId);
  if (!pos) return { wpEstimate: null, confidence: null, anomaly: null };

  const attr = pos.attributes ?? {};

  // CATEC: wp_reached is the WP index the autopilot confirmed
  if (attr.wp_reached !== undefined && attr.wp_reached !== null && attr.wp_reached !== 0) {
    const wpReached = Number(attr.wp_reached);
    if (!Number.isNaN(wpReached) && wpReached >= 0 && wpReached <= totalWp) {
      return { wpEstimate: wpReached, confidence: 'high', anomaly: null };
    }
  }

  // Future: DJI waypointV2 mission_state could map here
  return { wpEstimate: null, confidence: null, anomaly: null };
}

// ---------------------------------------------------------------------------
// Signal 3 — Time-based WP estimate
// Uses initTime + per-segment distance / speed to compute expected WP index.
// Works for any UAV type as long as speed and waypoints are known.
// Useful to detect missed WP detections due to telemetry gaps.
// ---------------------------------------------------------------------------
export function signalTimeEstimate(waypoints, routeAttributes, initTime, currentWp, totalWp) {
  if (!initTime || !waypoints?.length) return { wpEstimate: null, confidence: null, anomaly: null };

  const speed = routeAttributes?.max_vel ?? routeAttributes?.idle_vel ?? 5; // m/s fallback
  if (speed <= 0) return { wpEstimate: null, confidence: null, anomaly: null };

  // Build per-segment durations
  const segmentDurations = []; // seconds per leg
  for (let i = 0; i < waypoints.length - 1; i++) {
    const a = waypoints[i];
    const b = waypoints[i + 1];
    const [aLat, aLon] = Array.isArray(a.pos) ? a.pos : [a.pos.lat, a.pos.lon];
    const [bLat, bLon] = Array.isArray(b.pos) ? b.pos : [b.pos.lat, b.pos.lon];
    const legSpeed = b.speed ?? a.speed ?? speed;
    const dist = haversineMeters(aLat, aLon, bLat, bLon);
    segmentDurations.push(dist / legSpeed);
  }

  const totalDuration = segmentDurations.reduce((s, d) => s + d, 0);
  if (totalDuration <= 0) return { wpEstimate: null, confidence: null, anomaly: null };

  const elapsedSec = (Date.now() - new Date(initTime).getTime()) / 1000;
  if (elapsedSec < 0) return { wpEstimate: null, confidence: null, anomaly: null };

  // Walk through segments to find estimated WP
  let accumulated = 0;
  let estimatedWp = 0;
  for (let i = 0; i < segmentDurations.length; i++) {
    if (elapsedSec < accumulated + segmentDurations[i]) break;
    accumulated += segmentDurations[i];
    estimatedWp = i + 1;
  }
  estimatedWp = Math.min(estimatedWp, totalWp);

  // If time estimate is ahead of haversine-tracked WP by more than 2 → telemetry gap suspected
  const gap = estimatedWp - currentWp;
  const anomaly = gap >= 2 ? 'TELEMETRY_GAP' : null;

  logger.debug(`missionSignals[time] elapsed=${elapsedSec.toFixed(0)}s estimatedWp=${estimatedWp} currentWp=${currentWp} gap=${gap}`);

  return {
    wpEstimate: estimatedWp,
    confidence: gap < 2 ? 'medium' : 'low',
    anomaly,
  };
}

// ---------------------------------------------------------------------------
// Signal 4 — Spatial deviation from target WP
// Keeps a short rolling history of distances to detect sustained divergence.
// ---------------------------------------------------------------------------

// Per-device rolling distance history: { [deviceId]: number[] }
const _distHistory = {};
const DEVIATION_WINDOW = 5;       // last N samples
const DEVIATION_THRESHOLD_M = 30; // growing this much means diverging

export function signalDeviation(deviceId, currentLat, currentLon, targetWp) {
  if (!targetWp) return { wpEstimate: null, confidence: null, anomaly: null };

  const [tLat, tLon] = Array.isArray(targetWp.pos)
    ? targetWp.pos
    : [targetWp.pos.lat, targetWp.pos.lon];

  const dist = haversineMeters(currentLat, currentLon, tLat, tLon);

  if (!_distHistory[deviceId]) _distHistory[deviceId] = [];
  const hist = _distHistory[deviceId];
  hist.push(dist);
  if (hist.length > DEVIATION_WINDOW) hist.shift();

  if (hist.length < DEVIATION_WINDOW) {
    return { wpEstimate: null, confidence: null, anomaly: null };
  }

  // Check if distance is monotonically increasing over the window
  const isGrowing = hist.every((d, i) => i === 0 || d >= hist[i - 1]);
  const totalGrowth = hist[hist.length - 1] - hist[0];

  const anomaly = isGrowing && totalGrowth > DEVIATION_THRESHOLD_M ? 'DEVIATION' : null;

  logger.debug(`missionSignals[deviation] device=${deviceId} dist=${dist.toFixed(1)}m growth=${totalGrowth.toFixed(1)}m anomaly=${anomaly}`);

  return { wpEstimate: null, confidence: null, anomaly };
}

// ---------------------------------------------------------------------------
// Combine all signals into a single result
// Priority for wpEstimate: high > medium > low > null (haversine stays as base)
// Anomalies are merged — all detected ones are reported.
// ---------------------------------------------------------------------------
export function combineSignals(signals) {
  const priority = { high: 3, medium: 2, low: 1, null: 0 };

  let bestEstimate = null;
  let bestConfidence = null;
  const anomalies = new Set();

  for (const s of signals) {
    if (s.anomaly) s.anomaly.split(',').forEach((a) => anomalies.add(a));
    if (s.wpEstimate !== null) {
      const curr = priority[bestConfidence] ?? 0;
      const next = priority[s.confidence] ?? 0;
      if (next > curr) {
        bestEstimate = s.wpEstimate;
        bestConfidence = s.confidence;
      }
    }
  }

  return {
    wpEstimate: bestEstimate,
    confidence: bestConfidence,
    anomalies: [...anomalies],
  };
}

export function clearDeviationHistory(deviceId) {
  delete _distHistory[deviceId];
}
