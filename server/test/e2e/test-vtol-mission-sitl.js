#!/usr/bin/env node

/**
 * E2E: a VTOL mission over MAVLink against PX4 SITL, through the GCS manual flow.
 *
 * Builds a mission with fixed-wing geometry around where the vehicle is (relative heights:
 * the default altitude_ref), loads and commands it like the mission editor does
 * (POST /missions/load, POST /missions/command), and follows the flight from both sides:
 * the GCS (task status, wp_reached, vtolState) and PX4 itself (px4-listener). It fails on the
 * first step that does not happen and, if the vehicle is in the air, sends ReturnHome (Land if
 * that is refused) and waits for it to come down. The PX4 ULog of the flight is copied next to
 * the report: a SITL restart deletes it, and its console messages (quad-chute, transition) only
 * live there.
 *
 * Prerequisites:
 * - PX4 SITL VTOL on the ground and disarmed: make px4_sitl gz_standard_vtol
 * - GCS server on :4000 with MAVLINK_CONNECTION=true and a device for it: model standard_vtol,
 *   interface px4_mavlink, protocol mavlink (attributes.systemId = the SITL's MAV_SYS_ID)
 * - Nobody else commanding the vehicle while it runs (~5 min)
 *
 * Run:  cd multiuav_gcs/server && npm run test:e2e:vtol
 * Env:  GCS_URL (http://localhost:4000), DEVICE (vtol_px4), PX4_DIR (~/PX4-Autopilot),
 *       OUT_DIR (os tmpdir), ALT (40), LEG (500)
 */

import { execFile } from 'node:child_process';
import { promisify } from 'node:util';
import { copyFileSync, mkdirSync, readdirSync, statSync, writeFileSync } from 'node:fs';
import { homedir, tmpdir } from 'node:os';
import { join } from 'node:path';

const run = promisify(execFile);

const GCS = process.env.GCS_URL ?? 'http://localhost:4000';
const DEVICE = process.env.DEVICE ?? 'vtol_px4';
const PX4_DIR = process.env.PX4_DIR ?? join(homedir(), 'PX4-Autopilot');
const PX4_BIN = join(PX4_DIR, 'build/px4_sitl_default/bin');
const ULOG_DIR = join(PX4_DIR, 'build/px4_sitl_default/rootfs/fs/log');
const ALT = Number(process.env.ALT ?? 40); // m over the takeoff point
const LEG = Number(process.env.LEG ?? 500); // m, a fixed-wing leg
const OUT_DIR = process.env.OUT_DIR ?? join(tmpdir(), `vtol-e2e-${new Date().toISOString().replace(/[:.]/g, '-')}`);

const POLL_MS = 1000;
const FLIGHT_TIMEOUT_S = 600;
const STALL_S = 120; // airborne with no mission progress
const TRANSITION_S = 60; // a transition PX4 does not finish (its own VT_TRANS_TIMEOUT is ~15 s)
const MAX_POLL_ERRORS = 10; // the GCS or the SITL stopped answering
const RECOVERY_TIMEOUT_S = 300;

// PX4 enums read from px4-listener (vtol_vehicle_status / vehicle_status)
const PX4_VTOL_STATE = { 1: 'TRANSITION_TO_FW', 2: 'TRANSITION_TO_MC', 3: 'MC', 4: 'FW' };
const PX4_VEHICLE_TYPE = { 1: 'ROTARY_WING', 2: 'FIXED_WING' };

const t0 = Date.now();
const timeline = [];
const elapsed = () => ((Date.now() - t0) / 1000).toFixed(1);
const log = (kind, data) => {
  const entry = { t: Number(elapsed()), kind, ...data };
  timeline.push(entry);
  console.log(`[${elapsed().padStart(6)}s] ${kind} ${JSON.stringify(data)}`);
};
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// ─── GCS ──────────────────────────────────────────────────────────────────────

async function gcs(path, body) {
  const res = await fetch(`${GCS}/api${path}`, {
    method: body ? 'POST' : 'GET',
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  const text = await res.text();
  const data = text ? JSON.parse(text) : null;
  if (!res.ok) throw new Error(`${body ? 'POST' : 'GET'} ${path} → ${res.status} ${text}`);
  return data;
}

const positionOf = async (deviceId) => {
  const all = await gcs('/positions');
  return (Array.isArray(all) ? all : Object.values(all)).find((p) => p.deviceId === deviceId) ?? null;
};

async function taskOf(missionId) {
  const tasks = await gcs(`/missions/tasks?missionId=${missionId}`);
  return tasks?.[0] ?? null;
}

const sendCommand = (deviceId, type) => gcs('/commands/send', { deviceId, type });

// ─── PX4 (SITL console clients) ───────────────────────────────────────────────

async function listen(topic) {
  try {
    const { stdout } = await run(join(PX4_BIN, 'px4-listener'), [topic], { timeout: 4000 });
    const fields = {};
    for (const line of stdout.split('\n')) {
      const m = line.match(/^\s+(\w+): (.*)$/);
      if (m && !(m[1] in fields)) fields[m[1]] = m[2].trim();
    }
    return fields;
  } catch {
    return null;
  }
}

async function px4State() {
  const [vtol, status, mission, mlog] = await Promise.all([
    listen('vtol_vehicle_status'),
    listen('vehicle_status'),
    listen('mission_result'),
    listen('mavlink_log'),
  ]);
  return {
    vtolState: PX4_VTOL_STATE[vtol?.vehicle_vtol_state] ?? vtol?.vehicle_vtol_state ?? null,
    fwFailure: vtol?.fixed_wing_system_failure === 'True',
    vehicleType: PX4_VEHICLE_TYPE[status?.vehicle_type] ?? status?.vehicle_type ?? null,
    inTransition: status?.in_transition_mode === 'True',
    navState: status?.nav_state ?? null,
    armed: status?.arming_state === '2',
    missionValid: mission?.valid === 'True',
    seqCurrent: mission ? Number(mission.seq_current) : null,
    seqReached: mission ? Number(mission.seq_reached) : null,
    seqTotal: mission ? Number(mission.seq_total) : null,
    missionFinished: mission?.finished === 'True',
    // mavlink_log keeps the LAST message only (the full list comes from the ULog at the end), and
    // keeps it however old: one from before this run says nothing about this flight.
    lastMessage: isFromThisRun(mlog) ? mlog.text?.replace(/^"|"$/g, '') : null,
  };
}

// px4-listener prints `timestamp: 7504000 (703.46 seconds ago)`.
const messageAgeS = (field) => Number(field?.match(/\(([\d.]+) seconds ago\)/)?.[1] ?? Infinity);
const isFromThisRun = (msg) => msg != null && messageAgeS(msg.timestamp) < (Date.now() - t0) / 1000;

// ─── mission ──────────────────────────────────────────────────────────────────

// Flat-earth offset: the legs are a few hundred meters, far below where it matters.
const offset = ([lat, lon], east, north) => [
  lat + north / 111320,
  lon + east / (111320 * Math.cos((lat * Math.PI) / 180)),
];

function buildMission(device, home) {
  const at = (east, north, action) => ({ pos: [...offset(home, east, north), ALT], ...(action ? { action } : {}) });
  return {
    version: '4',
    name: `e2e VTOL ${new Date().toISOString()}`,
    description: 'test/e2e/test-vtol-mission-sitl.js',
    tasks: [
      {
        task_id: 'T1',
        device: device.name,
        action: 'VTOL_E2E',
        depends_on: [],
        // Auto landing: PX4's VTOL defaults (MIS_TKO_LAND_REQ=2) refuse a mission without one.
        params: { mode_yaw: 0, mode_trace: 2, mode_landing: 2, idle_vel: 5 },
        wp: [
          at(0, 0), // takeoff over the home, multicopter
          at(60, 0, { vtol_transition: 0 }), // → FW; PX4 first turns to the next waypoint, a long leg east
          at(60 + LEG, 0),
          at(60 + LEG, LEG * 0.8),
          at(60, LEG * 0.8, { vtol_transition: 1 }), // → MC
          at(0, 40), // multicopter again, then lands here
        ],
      },
    ],
  };
}

// ─── milestones ───────────────────────────────────────────────────────────────

// Either side reporting a transition counts (it can be shorter than a poll); a settled state
// (FW, MC again) must be seen by BOTH, which is also what checks the GCS's vtolState.
const either = (s, state) => s.px4.vtolState === state || s.gcs.vtolState === state;
const both = (s, state) => s.px4.vtolState === state && s.gcs.vtolState === state;
const onGround = (s) => s.gcs.armState === 'DISARMED' && s.gcs.flightPhase === 'ON_GROUND';

const MILESTONES = [
  ['armed', (s) => s.gcs.armState === 'ARMED' || s.px4.armed],
  ['airborne', (s) => s.gcs.flightPhase === 'IN_AIR'],
  ['transition to FW started', (s) => either(s, 'TRANSITION_TO_FW')],
  ['fixed wing', (s) => both(s, 'FW')],
  ['transition to MC started', (s, seen) => seen.has('fixed wing') && either(s, 'TRANSITION_TO_MC')],
  ['multicopter again', (s, seen) => seen.has('fixed wing') && both(s, 'MC')],
  ['landed and disarmed', (s, seen) => seen.has('airborne') && onGround(s)],
  ['task done', (s) => ['complete', 'end'].includes(s.task?.status)],
];

async function snapshot(deviceId, missionId) {
  const [pos, task, px4] = await Promise.all([positionOf(deviceId), taskOf(missionId), px4State()]);
  const a = pos?.attributes ?? {};
  return {
    gcs: {
      armState: a.armState,
      flightPhase: a.flightPhase,
      flightMode: a.flightMode,
      vtolState: a.vtolState ?? null,
      wpReached: a.wp_reached,
      altitude: pos?.altitude,
      speed: pos?.speed,
    },
    task: task && { status: task.status, currentWp: task.currentWp, totalWp: task.totalWp, error: task.errorMessage },
    px4,
  };
}

// What changed, as one line: the timeline stays readable at 1 Hz.
function diff(prev, cur) {
  const out = {};
  for (const side of ['gcs', 'task', 'px4']) {
    for (const [k, v] of Object.entries(cur[side] ?? {})) {
      if (['altitude', 'speed'].includes(k)) continue;
      if (prev?.[side]?.[k] !== v) out[`${side}.${k}`] = v;
    }
  }
  return out;
}

// The first reason to stop the flight, or null.
function failureOf(s, { seen, transitionSince, lastProgressAt, deadline }) {
  const now = Date.now();
  if (['error', 'cancelled', 'skipped'].includes(s.task?.status)) {
    return `task ${s.task.status}: ${s.task.error ?? ''}`;
  }
  if (/quad-?chute/i.test(s.px4.lastMessage ?? '') || s.px4.fwFailure) return `quad-chute: ${s.px4.lastMessage}`;
  if (transitionSince && now - transitionSince > TRANSITION_S * 1000) {
    return `transition (${s.px4.vtolState}) not finished after ${TRANSITION_S}s`;
  }
  if (s.gcs.flightPhase === 'IN_AIR' && now - lastProgressAt > STALL_S * 1000) {
    return `no mission progress for ${STALL_S}s at seq ${s.px4.seqCurrent}/${s.px4.seqTotal} (vtol ${s.px4.vtolState})`;
  }
  if (seen.has('airborne') && s.gcs.armState === 'DISARMED' && !seen.has('multicopter again')) {
    return 'disarmed before completing the VTOL cycle';
  }
  if (now > deadline) return `timeout ${FLIGHT_TIMEOUT_S}s`;
  return null;
}

// ─── recovery ─────────────────────────────────────────────────────────────────

async function recover(deviceId, missionId, reason) {
  log('FAILURE', { reason });
  const s = await snapshot(deviceId, missionId);
  if (s.gcs.flightPhase !== 'IN_AIR' && s.gcs.armState !== 'ARMED') {
    log('recovery', { action: 'none', why: 'on the ground and disarmed' });
    return;
  }
  for (const type of ['ReturnHome', 'Land']) {
    try {
      const r = await sendCommand(deviceId, type);
      log('recovery', { action: type, state: r?.state, msg: r?.msg });
      if (r?.state === 'success') break;
    } catch (err) {
      log('recovery', { action: type, error: err.message });
    }
  }
  const deadline = Date.now() + RECOVERY_TIMEOUT_S * 1000;
  let prev = null;
  while (Date.now() < deadline) {
    const cur = await snapshot(deviceId, missionId);
    const d = diff(prev, cur);
    if (Object.keys(d).length) log('state', d);
    prev = cur;
    if (cur.gcs.armState === 'DISARMED' && cur.gcs.flightPhase === 'ON_GROUND') {
      log('recovery', { action: 'done', why: 'landed and disarmed' });
      return;
    }
    await sleep(POLL_MS);
  }
  log('recovery', { action: 'gave up', why: `still not on the ground after ${RECOVERY_TIMEOUT_S}s: CHECK THE SIM` });
}

// ─── ULog ─────────────────────────────────────────────────────────────────────

function newestUlogSince(sinceMs) {
  let best = null;
  try {
    for (const day of readdirSync(ULOG_DIR)) {
      for (const f of readdirSync(join(ULOG_DIR, day))) {
        if (!f.endsWith('.ulg')) continue;
        const path = join(ULOG_DIR, day, f);
        const { mtimeMs } = statSync(path);
        if (mtimeMs >= sinceMs && (!best || mtimeMs > best.mtimeMs)) best = { path, mtimeMs };
      }
    }
  } catch {
    return null;
  }
  return best?.path ?? null;
}

async function saveUlog() {
  await sleep(3000); // the logger closes the file shortly after disarming
  const ulog = newestUlogSince(t0);
  if (!ulog) return log('ulog', { saved: false, why: `no .ulg newer than the run under ${ULOG_DIR}` });
  const copy = join(OUT_DIR, 'flight.ulg');
  copyFileSync(ulog, copy);
  log('ulog', { saved: copy, from: ulog });
  try {
    const { stdout } = await run('ulog_messages', [copy], { timeout: 30000, maxBuffer: 16 * 1024 * 1024 });
    writeFileSync(join(OUT_DIR, 'px4_messages.txt'), stdout);
    const interesting = stdout
      .split('\n')
      .filter((l) => /transition|quad-?chute|vtol|mission|reject|failsafe|fixed.?wing|land|takeoff/i.test(l));
    log('px4 messages', { file: join(OUT_DIR, 'px4_messages.txt'), relevant: interesting.slice(0, 60) });
  } catch (err) {
    log('px4 messages', { error: `ulog_messages failed: ${err.message}` });
  }
}

// ─── main ─────────────────────────────────────────────────────────────────────

async function main() {
  mkdirSync(OUT_DIR, { recursive: true });
  console.log(`VTOL e2e → ${OUT_DIR}`);

  const device = (await gcs('/devices')).find((d) => d.name === DEVICE);
  if (!device) throw new Error(`device '${DEVICE}' not found`);
  const vehicleType = device.platform?.vehicleType;
  log('device', { id: device.id, model: device.deviceModel, category: device.category, vehicleType });
  if (vehicleType !== 'vtol') throw new Error(`device '${DEVICE}' is not a VTOL (${vehicleType})`);

  const pos = await positionOf(device.id);
  const a = pos?.attributes ?? {};
  if (!pos?.latitude) throw new Error('no position from the vehicle');
  if (a.armState !== 'DISARMED' || a.flightPhase !== 'ON_GROUND') {
    const state = `armState=${a.armState}, flightPhase=${a.flightPhase}`;
    throw new Error(`the vehicle must be on the ground and disarmed (${state})`);
  }
  const px4 = await px4State();
  if (px4.vtolState == null) throw new Error(`px4-listener does not answer (${PX4_BIN}): is the SITL running?`);
  const home = a.home ?? [pos.latitude, pos.longitude];
  log('start', { home, alt: ALT, leg: LEG, px4VtolState: px4.vtolState });

  const mission = buildMission(device, home);
  writeFileSync(join(OUT_DIR, 'mission.json'), JSON.stringify(mission, null, 2));

  const loaded = await gcs('/missions/load', mission);
  log('load', loaded);
  if (!loaded?.results?.every((r) => r.state === 'success')) {
    throw new Error(`load failed: ${JSON.stringify(loaded?.results)}`);
  }
  const { missionId } = loaded;

  const before = await px4State();
  log('px4 mission', { valid: before.missionValid, seqTotal: before.seqTotal });
  if (!before.missionValid) {
    throw new Error('PX4 marks the uploaded mission invalid (see px4_messages.txt / its console)');
  }

  const commanded = await gcs('/missions/command', { missionId });
  log('command', commanded);

  const seen = new Set();
  let prev = null;
  let lastProgressAt = Date.now();
  let lastSeq = null;
  let failure = null;
  const deadline = Date.now() + FLIGHT_TIMEOUT_S * 1000;

  let transitionSince = null;
  let pollErrors = 0;

  while (!failure) {
    let s;
    try {
      s = await snapshot(device.id, missionId);
      pollErrors = 0;
    } catch (err) {
      log('poll error', { error: err.message });
      if (++pollErrors >= MAX_POLL_ERRORS) failure = `${MAX_POLL_ERRORS} polls in a row failed: ${err.message}`;
      await sleep(POLL_MS);
      continue;
    }
    const d = diff(prev, s);
    if (Object.keys(d).length) log('state', d);
    prev = s;

    for (const [name, check] of MILESTONES) {
      if (!seen.has(name) && check(s, seen)) {
        seen.add(name);
        log('MILESTONE', { name, alt: s.gcs.altitude, speed: s.gcs.speed });
      }
    }
    if (s.px4.seqCurrent !== lastSeq || s.px4.inTransition) {
      lastSeq = s.px4.seqCurrent;
      lastProgressAt = Date.now();
    }
    transitionSince = s.px4.inTransition ? (transitionSince ?? Date.now()) : null;

    if (seen.has('task done') && seen.has('landed and disarmed')) break;
    failure = failureOf(s, { seen, transitionSince, lastProgressAt, deadline });
    if (!failure) await sleep(POLL_MS);
  }

  if (failure) await recover(device.id, missionId, failure);
  await saveUlog();

  const missing = MILESTONES.map(([name]) => name).filter((n) => !seen.has(n));
  const pass = !failure && missing.length === 0;
  const report = { pass, failure, missing, milestones: [...seen], outDir: OUT_DIR, timeline };
  writeFileSync(join(OUT_DIR, 'report.json'), JSON.stringify(report, null, 2));
  const why = [failure, missing.length ? `missing: ${missing.join(', ')}` : null].filter(Boolean).join(' — ');
  console.log(`\n${pass ? 'PASS' : 'FAIL'}${why ? ` — ${why}` : ''}`);
  console.log(`report: ${join(OUT_DIR, 'report.json')}`);
  process.exit(report.pass ? 0 : 1);
}

main().catch((err) => {
  log('ABORT', { error: err.message });
  writeFileSync(join(OUT_DIR, 'report.json'), JSON.stringify({ pass: false, abort: err.message, timeline }, null, 2));
  console.log(`\nFAIL — ${err.message}\nreport: ${join(OUT_DIR, 'report.json')}`);
  process.exit(1);
});
