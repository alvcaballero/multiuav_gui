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
import { normalizeMission, TaskGraphError } from './taskGraph.js';
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

/**
 * What one task asks of its device that the device cannot do. Synchronous: the device is resolved.
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
