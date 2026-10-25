// ─── Mission validation: what each task asks of its device ───────────────────
//
// One validator for every path that loads a mission (manual load, automatic flow, the LoadMission
// command the MCP sends) and for POST /missions/check, which the mission editor calls from its
// Validate button: the rules live here once, never in the client.
//
// A mission may only carry what its interface encodes AND its vehicle can do
// (VEHICLE_PLATFORMS_PLAN.md › F5 · Contenido de la misión): an action, route param or waypoint param
// owned by a capability the device lacks (capabilities.yaml `mission`), meant for other vehicle types
// (mission_schema.yaml `vehicleTypes`), or that its interface does not encode, is an ERROR and the load
// is refused, with where (task, waypoint, field) and why. A device that does not exist is a WARNING:
// the load goes on and that task fails, so its dependents are skipped (the flows' existing rule).
import { normalizeMission, TaskGraphError, ALTITUDE_REF } from './taskGraph.js';
import { VEHICLE_TYPE } from '../../config/status.js';
import { categoryModel } from '../category.js';
import { deviceModelsModel } from '../deviceModels/deviceModels.js';
import { capabilityOfMissionItem } from '../capabilities/capabilityCatalog.js';
import { commandRegistry } from '../commands/commandRegistry.js';
import { CommandType } from '../../config/commandTypes.js';

export const PROBLEM_LEVEL = Object.freeze({ ERROR: 'error', WARNING: 'warning' });

export class MissionValidationError extends Error {
  constructor(problems) {
    const errors = problems.filter((p) => p.level === PROBLEM_LEVEL.ERROR).map(formatProblem);
    super(`Invalid mission: ${errors.join('; ')}`);
    this.name = 'MissionValidationError';
    this.errors = errors;
    this.problems = problems;
    this.status = 400;
  }
}

/** `T1 · wp 3 · action.gimbal: …` — `wp` is 0-based in the problem, shown 1-based. */
export function formatProblem({ task, wp, field, message }) {
  const where = [task, wp != null ? `wp ${wp + 1}` : null, field].filter(Boolean).join(' · ');
  return where ? `${where}: ${message}` : message;
}

const problem = (level, task, fields) => ({
  level,
  task: task?.task_id ?? null,
  device: task?.device ?? null,
  ...fields,
});
const error = (task, fields) => problem(PROBLEM_LEVEL.ERROR, task, fields);

// Why `key` (of `kind`) is not in the device's profile, read from the same catalogs that removed it.
function whyNot(kind, key, device, interfaceProfile) {
  const PROFILE_KEY = { actions: 'actions', params: 'params', waypoint_params: 'waypointParams' };
  if (!categoryModel.missionKeys()[kind]?.has(key)) {
    return `'${key}' is not a mission ${kind.replace('_', ' ')} the GCS knows`;
  }
  const def = interfaceProfile?.[PROFILE_KEY[kind]]?.[key];
  if (!def) return `its interface '${device.category}' does not encode '${key}'`;
  const capability = capabilityOfMissionItem(deviceModelsModel.getCapabilities(), kind, key);
  if (capability && !deviceModelsModel.allowsMissionItem(device, kind, key)) {
    return `'${key}' needs capability '${capability}', which ${device.name} (model ${device.deviceModel}) does not have`;
  }
  const { vehicleType } = deviceModelsModel.missionRestrictionOf(device);
  return `'${key}' is only for ${def.vehicleTypes?.join(', ')}; ${device.name} is a ${vehicleType}`;
}

// The value of a param or an action payload against its definition: a select option the device is
// offered (the encoder's and the vehicle type's), a number in range. A flag action takes no value.
function valueProblem(def, value) {
  if (!def || value == null) return null;
  if (def.type === 'select') {
    const options = def.options ?? [];
    return options.some((o) => o.value === Number(value))
      ? null
      : `${value} is not an option here (${options.map((o) => `${o.value} ${o.name}`).join(', ') || 'none'})`;
  }
  if (def.type === 'number') {
    const n = Number(value);
    if (!Number.isFinite(n)) return `${JSON.stringify(value)} is not a number`;
    if (def.min != null && n < def.min) return `${n} is below the minimum ${def.min}`;
    if (def.max != null && n > def.max) return `${n} is above the maximum ${def.max}`;
  }
  return null;
}

// ─── Geometry on the wing (VTOL, fixed wing): WARNINGS ─────────────────────────
//
// Whether the route can be flown on the wing, against the device model's numbers (`speeds.fixed_wing`,
// `limits`, `vtol`: estimates from the autopilot's parameters, so a warning, never a refusal). A
// check without its numbers in the model is skipped, never guessed. Four checks:
//   1. the leg after a transition to the wing must fit the transition (airspeed × duration);
//   2. each turn on the wing takes R·tan(θ/2) of the leg before and after the waypoint;
//   3. the leg after a transition back to the rotors must fit the braking (v² / 2a);
//   4. no waypoint flown on the wing below `min_fixed_wing_altitude` (relative heights only).
const EARTH_M_PER_DEG = 111320;
const SHARPEST_FW_TURN_DEG = 150; // past this a wing cannot turn on the waypoint: it needs a loiter

// Flat-earth: the legs are a few km at most.
function legOf(a, b) {
  const east = (b[1] - a[1]) * EARTH_M_PER_DEG * Math.cos((a[0] * Math.PI) / 180);
  const north = (b[0] - a[0]) * EARTH_M_PER_DEG;
  return { length: Math.hypot(east, north), heading: (Math.atan2(east, north) * 180) / Math.PI };
}

const turnDeg = (h1, h2) => Math.abs(((((h2 - h1) % 360) + 540) % 360) - 180);
const round = (m) => Math.round(m);

function geometryProblems(task, device) {
  const model = deviceModelsModel.get(device.deviceModel);
  const vehicleType = model?.vehicleType;
  if (vehicleType !== VEHICLE_TYPE.VTOL && vehicleType !== VEHICLE_TYPE.FIXED_WING) return [];
  const wps = Object.values(task.wp ?? {}).filter((wp) => Array.isArray(wp?.pos));
  if (wps.length < 2) return [];

  const speed = model.speeds?.fixed_wing;
  const radius = model.limits?.min_turn_radius;
  const minAltitude = model.limits?.min_fixed_wing_altitude;
  const { transition_airspeed: transitionSpeed, front_transition_duration: transitionTime } = model.vtol ?? {};
  const decel = model.vtol?.back_transition_deceleration;
  const frontRoom = transitionSpeed && transitionTime ? transitionSpeed * transitionTime : null;
  const brakeRoom = speed && decel ? (speed * speed) / (2 * decel) : null;

  const warn = (wp, field, message) => problem(PROBLEM_LEVEL.WARNING, task, { wp, field, message });
  const problems = [];

  // How each leg is flown: a fixed wing always on the wing; a VTOL on its rotors until a
  // vtol_transition to the wing (0), and back on a transition to the rotors (1), after its waypoint.
  let onWing = vehicleType === VEHICLE_TYPE.FIXED_WING;
  const legs = [];
  for (let i = 0; i < wps.length - 1; i += 1) {
    const transition = wps[i].action?.vtol_transition;
    const toWing = transition != null && Number(transition) === 0;
    const toRotors = transition != null && Number(transition) === 1;
    if (toWing) onWing = true;
    if (toRotors) onWing = false;
    legs.push({ ...legOf(wps[i].pos, wps[i + 1].pos), onWing, toWing, toRotors, needed: 0, why: [] });
  }
  if (Number(wps.at(-1).action?.vtol_transition) === 0) {
    const message = 'a transition to the wing at the last waypoint: nothing is flown on it';
    problems.push(warn(wps.length - 1, 'action.vtol_transition', message));
  }

  // 1. room for the transition to the wing, at the start of its leg
  for (const leg of legs) {
    if (leg.toWing && frontRoom) {
      leg.needed += frontRoom;
      leg.why.push(`~${round(frontRoom)} m to reach ${transitionSpeed} m/s on the wing`);
    }
  }
  // 2. turns on the wing: both legs of waypoint k on the wing
  if (radius) {
    for (let k = 1; k < legs.length; k += 1) {
      if (!legs[k - 1].onWing || !legs[k].onWing) continue;
      const theta = turnDeg(legs[k - 1].heading, legs[k].heading);
      if (theta > SHARPEST_FW_TURN_DEG) {
        const message = `a ${round(theta)}° turn is too sharp to fly on the wing (radius ${radius} m): it needs a loiter`;
        problems.push(warn(k, 'geometry', message));
        continue;
      }
      const room = radius * Math.tan((theta * Math.PI) / 360);
      if (room < 1) continue;
      legs[k - 1].needed += room;
      legs[k - 1].why.push(`~${round(room)} m for the ${round(theta)}° turn at wp ${k + 1}`);
      legs[k].needed += room;
      legs[k].why.push(`~${round(room)} m for the ${round(theta)}° turn at wp ${k + 1}`);
    }
  }
  for (const [i, leg] of legs.entries()) {
    if (leg.onWing && leg.length < leg.needed) {
      const message =
        `the leg to wp ${i + 2} is ${round(leg.length)} m on the wing; ` +
        `it needs ~${round(leg.needed)} m (${leg.why.join(', ')})`;
      problems.push(warn(i, 'geometry', message));
    }
  }
  // 3. braking back to the rotors, on the leg after the transition
  for (const [i, leg] of legs.entries()) {
    if (leg.toRotors && brakeRoom && leg.length < brakeRoom) {
      problems.push(
        warn(
          i,
          'action.vtol_transition',
          `back on the rotors it brakes from ${speed} m/s in ~${round(brakeRoom)} m, but wp ${i + 2} is ${round(leg.length)} m away: it overshoots and comes back`
        )
      );
    }
  }
  // 4. height on the wing: every waypoint that starts or ends a leg on the wing
  if (minAltitude && (task.altitude_ref ?? ALTITUDE_REF.RELATIVE) === ALTITUDE_REF.RELATIVE) {
    for (const [i, wp] of wps.entries()) {
      const onWingHere = legs[i - 1]?.onWing || legs[i]?.onWing;
      if (onWingHere && Number(wp.pos[2]) < minAltitude) {
        const message = `${wp.pos[2]} m is low to fly on the wing (minimum ${minAltitude} m over the takeoff)`;
        problems.push(warn(i, 'pos', message));
      }
    }
  }
  return problems;
}

/**
 * What one task asks of its device that the device cannot do (ERRORS), and where its route looks
 * unflyable on the wing (WARNINGS). Synchronous: the device is resolved.
 * @param {object} task a normalized task (taskGraph.normalizeMission)
 * @param {object} device the device it flies on ({ name, category, deviceModel, attributes })
 * @returns {object[]} problems, `{ level, task, device, wp, field, message }`
 */
export function taskProblems(task, device) {
  const deviceError = (message) => [error(task, { wp: null, field: 'device', message })];
  const iface = categoryModel.getCategory(device.category);
  if (!iface) return deviceError(`${device.name} has an unknown interface '${device.category}'`);
  if (!commandRegistry.resolveCommands(iface).commands.has(CommandType.LOAD_MISSION)) {
    return deviceError(`the interface '${device.category}' of ${device.name} loads no missions`);
  }
  if (!deviceModelsModel.allowsCommand(device, CommandType.LOAD_MISSION)) {
    return deviceError(`${device.name} does not fly missions (it lacks capability 'autonomous_mission')`);
  }

  const interfaceProfile = categoryModel.missionProfile(device.category);
  const profile = categoryModel.missionProfile(device.category, deviceModelsModel.missionRestrictionOf(device));
  const problems = [];
  const check = (kind, key, value, defs, location) => {
    const def = defs[key];
    if (!def) return problems.push(error(task, { ...location, message: whyNot(kind, key, device, interfaceProfile) }));
    const why = valueProblem(kind === 'actions' ? def.payload : def, value);
    if (why) problems.push(error(task, { ...location, message: why }));
  };

  for (const [key, value] of Object.entries(task.params ?? {})) {
    check('params', key, value, profile.params, { wp: null, field: `params.${key}` });
  }
  const waypointParamKeys = categoryModel.missionKeys().waypoint_params;
  for (const [i, wp] of Object.values(task.wp ?? {}).entries()) {
    for (const [key, value] of Object.entries(wp?.action ?? {})) {
      check('actions', key, value, profile.actions, { wp: i, field: `action.${key}` });
    }
    for (const key of waypointParamKeys) {
      if (wp?.[key] != null) check('waypoint_params', key, wp[key], profile.waypointParams, { wp: i, field: key });
    }
  }
  problems.push(...geometryProblems(task, device));
  return problems;
}

/**
 * Every problem of a mission, any shape normalizeMission takes.
 * @param {object} missionData
 * @param {(name: string) => Promise<object|null>} findDevice
 * @returns {Promise<{ ok: boolean, problems: object[] }>} ok = no ERROR (warnings do not block)
 */
export async function missionProblems(missionData, findDevice) {
  let tasks;
  try {
    ({ tasks } = normalizeMission(missionData));
  } catch (err) {
    if (!(err instanceof TaskGraphError)) throw err;
    const problems = err.errors.map((message) => error(null, { wp: null, field: 'tasks', message }));
    return { ok: false, problems };
  }
  const problems = [];
  for (const task of tasks) {
    const device = await findDevice(task.device);
    if (!device) {
      const message = `device '${task.device}' does not exist: this task will fail and its dependents be skipped`;
      problems.push(problem(PROBLEM_LEVEL.WARNING, task, { wp: null, field: 'device', message }));
      continue;
    }
    problems.push(...taskProblems(task, device));
  }
  return { ok: !problems.some((p) => p.level === PROBLEM_LEVEL.ERROR), problems };
}

/** Throws MissionValidationError (400) when the mission has an ERROR; returns the warnings. */
export async function assertMissionValid(missionData, findDevice) {
  const { ok, problems } = await missionProblems(missionData, findDevice);
  if (!ok) throw new MissionValidationError(problems);
  return problems;
}
