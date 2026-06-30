import { categoryModel } from '../category.js';
import { missionLogger as logger } from '../../common/logger.js';
import { CONFIG_SYMBOLS } from '../ros/missionSymbols.js';

// ─── ConfigMission symbol → firmware number translation ───────────────────────
//
// CONFIG_SYMBOLS (in ros/missionSymbols.js) maps each CANONICAL SYMBOL (the `key`
// in mission_schema.yaml) to the aerialcore_common/ConfigMission firmware number.
// A symbol absent from that table means this firmware doesn't support the mode.

// symbol → ConfigMission firmware number. Throws if asked for a symbol this family
// doesn't map (should never happen: the catalog is filtered to supported options).
export function toConfigValue(group, symbol) {
  const n = CONFIG_SYMBOLS[group]?.[symbol];
  if (n === undefined) throw new RangeError(`MissionToRos: unmapped symbol ${group}.${symbol}`);
  return n;
}

// Resolves a route attribute (wire number) to its ConfigMission firmware number
// via the catalog symbol: number → symbol (catalog) → number (ConfigMission table).
function configParamFromValue(group, value) {
  const symbol = categoryModel.symbolForValue(group, value);
  if (symbol == null) throw new RangeError(`MissionToRos: no symbol for ${group}=${value}`);
  return toConfigValue(group, symbol);
}

// Defaults come from the mission_schema YAML, resolved per robot category (SSOT).
// route.attributes overrides any default the user explicitly set.
function extractRouteAttributes(routeAttributes, uavType) {
  const defaults = categoryModel.getAttributesDefaults(uavType);
  return { ...defaults, ...routeAttributes };
}

// ConfigMission transmits domain values UNCHANGED (no unit transform — this is the
// family's contract). The action gate uses `payload` (number action) vs flag.
function buildWaypointActions(wpAction, categoryActions) {
  const action_array = Array(10).fill(0);
  const param_array = Array(10).fill(0);
  Object.keys(wpAction).forEach((action_val, index) => {
    const found = categoryActions.find((el) => el.name === action_val);
    if (!found) {
      // The category's profile doesn't support this action — surface it instead
      // of silently dropping (e.g. focus/zoom on a non-PSDK robot).
      logger.warn(`MissionDecoder: action '${action_val}' not supported by this category, skipping`);
      return;
    }
    action_array[index] = Number(found.id);
    param_array[index] = found.payload != null ? Number(wpAction[action_val]) : 0;
  });
  return { action_array, param_array };
}

function transformWaypoints(waypoints, idle_vel, categoryActions) {
  const wp_command = [];
  const yaw_pos = [];
  const speed_pos = [];
  const gimbal_pos = [];
  const action_matrix = [];
  const param_matrix = [];

  for (const item of Object.values(waypoints)) {
    logger.debug(`wp action: ${JSON.stringify(item.action ?? 'no action')}`);

    const pos = {
      latitude: item.pos[0],
      longitude: item.pos[1],
      altitude: item.pos[2],
    };
    const yaw = item.hasOwnProperty('yaw') ? item.yaw : 0;
    const speed = item.hasOwnProperty('speed') ? item.speed : idle_vel;
    const gimbal = item.hasOwnProperty('gimbal') ? item.gimbal : 0;

    let action_array = Array(10).fill(0);
    let param_array = Array(10).fill(0);
    if (item.hasOwnProperty('action')) {
      ({ action_array, param_array } = buildWaypointActions(item.action, categoryActions));
    }

    wp_command.push(pos);
    yaw_pos.push(yaw);
    speed_pos.push(speed);
    gimbal_pos.push(gimbal);
    action_matrix.push(action_array);
    param_matrix.push(param_array);
  }

  return { wp_command, yaw_pos, speed_pos, gimbal_pos, action_matrix, param_matrix };
}

// ─── Public API ───────────────────────────────────────────────────────────────

// Decodes a raw route object into the internal normalized mission format.
// route.uav_type must be present so categoryModel can resolve action IDs.
export function decodeMissionRoute(route) {
  const { idle_vel, max_vel, mode_yaw, mode_gimbal, mode_trace, mode_landing } = extractRouteAttributes(
    route.attributes ?? {},
    route.uav_type
  );

  // categoryModel.getActions is synchronous — reads from in-memory YAML
  const categoryActions = categoryModel.getActions({ type: route.uav_type });

  const { wp_command, yaw_pos, speed_pos, gimbal_pos, action_matrix, param_matrix } = transformWaypoints(
    route.wp,
    idle_vel,
    categoryActions
  );

  return {
    waypoint: wp_command,
    maxVel: max_vel,
    idleVel: idle_vel,
    yaw: yaw_pos,
    speed: speed_pos,
    gimbalPitch: gimbal_pos,
    // Modes resolved wire number → catalog symbol → ConfigMission firmware number.
    yawMode: configParamFromValue('mode_yaw', mode_yaw),
    traceMode: configParamFromValue('mode_trace', mode_trace),
    finishAction: configParamFromValue('mode_landing', mode_landing),
    // mode_gimbal has its own scheme (GIMBAL_MODE_*); passed raw until mapped.
    gimbalPitchMode: mode_gimbal,
    commandList: action_matrix,
    commandParameter: param_matrix,
  };
}
