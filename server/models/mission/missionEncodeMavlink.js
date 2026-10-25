import { categoryModel } from '../category.js';
import { missionLogger as logger } from '../../common/logger.js';
import { MAVLINK_SYMBOLS, MAVLINK_MISSION } from './missionSymbols.js';
import { ALTITUDE_REF } from './taskGraph.js';

// ─── task → MAVLink mission (PX4 / ArduPilot) ─────────────────────────────────
//
// A task (taskGraph.js) becomes the item list the MAVLink mission protocol uploads
// (MavlinkServer._uploadMission). Positions are geodetic, pos = [lat, lon, alt], and what `alt`
// means is the task's `altitude_ref` (taskGraph ALTITUDE_REF): AMSL → MAV_FRAME_GLOBAL; relative,
// the default (the mission editor's "Height") → MAV_FRAME_GLOBAL_RELATIVE_ALT, over the home the
// autopilot itself holds. Sending an editor height as AMSL put the takeoff at 5 m above sea level
// with the home at 488 m: PX4 refused it ("Takeoff altitude below home altitude"), and ArduPilot,
// which has no such check on waypoints, would have been sent below the ground.
//
// The items are NOT one per waypoint: speed changes, gimbal, camera and ArduPilot's home slot
// are items too. `wpSeqs[i]` is the seq of waypoint i: MISSION_ITEM_REACHED reports seqs, and
// without this map `wp_reached` would count the extra items as waypoints and the task graph
// would complete early, or never.
//
// Dialect differences (MAVLINK_PLAN.md › F4):
//   - ArduPilot: seq 0 is the HOME, overwritten by the vehicle. A placeholder goes there.
//   - PX4: item 0 is flagged `current`, or the mission resumes where the last one ended.
//   - Waypoint yaw: PX4 reads NAV_WAYPOINT param4 (deg). ArduCopter ignores it: a CONDITION_YAW
//     item follows the waypoint.
//   - ArduPilot rejects the whole upload if a mission param is NaN where it does not allow it
//     (only param4 of NAV_WAYPOINT / NAV_TAKEOFF / NAV_LAND, and a few others).

export const MAV_CMD_MISSION = Object.freeze({
  NAV_WAYPOINT: 16,
  NAV_RETURN_TO_LAUNCH: 20,
  NAV_LAND: 21,
  NAV_TAKEOFF: 22,
  CONDITION_YAW: 115,
  DO_CHANGE_SPEED: 178,
  DO_GIMBAL_MANAGER_PITCHYAW: 1000,
  IMAGE_START_CAPTURE: 2000,
  VIDEO_START_CAPTURE: 2500,
  VIDEO_STOP_CAPTURE: 2501,
});

export const MAV_FRAME_MISSION_ITEM = Object.freeze({
  GLOBAL: 0, // AMSL
  MISSION: 2, // not a position
  GLOBAL_RELATIVE_ALT: 3, // altitude over the home
});

const SPEED_TYPE_GROUND = 1;
const THROTTLE_UNCHANGED = -1;

/** What the vehicle does after the last waypoint. */
export const MISSION_FINISH = Object.freeze({ HOVER: 'hover', HOME: 'home', LAND: 'land' });

const FINISH_OF = {
  MISSION_FINISHED_NO_ACTION: MISSION_FINISH.HOVER,
  MISSION_FINISHED_GO_HOME: MISSION_FINISH.HOME,
  MISSION_FINISHED_AUTO_LANDING: MISSION_FINISH.LAND,
};

// wire number → catalog symbol, refusing an option this family does not implement (the UI
// filters them out, but a planner or the LLM may still send one).
function symbolOf(group, value) {
  const symbol = categoryModel.symbolForValue(group, value);
  if (symbol == null || !(symbol in (MAVLINK_SYMBOLS[group] ?? {}))) {
    throw new RangeError(`MAVLink mission: ${group}=${value} (${symbol ?? 'unknown'}) is not supported`);
  }
  return symbol;
}

const FRAME_OF_ALTITUDE = {
  [ALTITUDE_REF.AMSL]: MAV_FRAME_MISSION_ITEM.GLOBAL,
  [ALTITUDE_REF.RELATIVE]: MAV_FRAME_MISSION_ITEM.GLOBAL_RELATIVE_ALT,
};
const positionAt = (frame) => (pos) => ({ frame, lat: pos[0], lon: pos[1], alt: pos[2] });
const notAt = () => ({ frame: MAV_FRAME_MISSION_ITEM.MISSION, x: 0, y: 0, z: 0 });

const changeSpeed = (speed) => ({ command: MAV_CMD_MISSION.DO_CHANGE_SPEED, params: [SPEED_TYPE_GROUND, speed, THROTTLE_UNCHANGED, 0], ...notAt() });
const conditionYaw = (deg) => ({ command: MAV_CMD_MISSION.CONDITION_YAW, params: [((deg % 360) + 360) % 360, 0, 0, 0], ...notAt() });
// x = GIMBAL_MANAGER_FLAGS (0: yaw follows the vehicle), z = gimbal device 0 (all).
// PX4: yaw and rates NaN = "pitch only, angle control" (with rate 0 PX4 does not move it).
// ArduPilot refuses NaN in every mission param it does not list (AP_Mission::sanity_check_params:
// INVALID_PARAM2): yaw 0 (ahead) and rates 0, which ArduPilot reads as angle control.
const gimbalPitch = (pitch, isArdupilot) => ({
  command: MAV_CMD_MISSION.DO_GIMBAL_MANAGER_PITCHYAW,
  params: isArdupilot ? [pitch, 0, 0, 0] : [pitch, NaN, NaN, NaN],
  ...notAt(),
});
const photo = () => ({ command: MAV_CMD_MISSION.IMAGE_START_CAPTURE, params: [0, 0, 1, 0], ...notAt() });
const videoStart = () => ({ command: MAV_CMD_MISSION.VIDEO_START_CAPTURE, params: [0, 0, 0, 0], ...notAt() });
const videoStop = () => ({ command: MAV_CMD_MISSION.VIDEO_STOP_CAPTURE, params: [0, 0, 0, 0], ...notAt() });

/**
 * @param {object} task a task with `uav_type` (commandRegistry.missionTaskForDevice)
 * @param {string} dialect MAVLINK_MISSION.*
 * @returns {{kind: 'mission', items: object[], wpSeqs: number[], finish: string}}
 */
export function encodeMavlinkMission(task, dialect) {
  if (!Object.values(MAVLINK_MISSION).includes(dialect)) throw new Error(`unknown MAVLink mission dialect '${dialect}'`);
  const isArdupilot = dialect === MAVLINK_MISSION.ARDUPILOT;
  const waypoints = Object.values(task.wp ?? {});
  if (waypoints.length === 0) throw new Error(`MAVLink mission: task ${task.task_id} has no waypoints`);
  // taskGraph already refuses an unknown altitude_ref; this keeps a task built elsewhere from
  // flying it as relative.
  const frame = FRAME_OF_ALTITUDE[task.altitude_ref ?? ALTITUDE_REF.RELATIVE];
  if (frame == null) {
    throw new Error(`MAVLink mission: task ${task.task_id} has an unknown altitude_ref '${task.altitude_ref}'`);
  }
  const at = positionAt(frame);

  const params = { ...categoryModel.getAttributesDefaults(task.uav_type), ...(task.params ?? {}) };
  const yawMode = symbolOf('mode_yaw', params.mode_yaw);
  symbolOf('mode_trace', params.mode_trace);
  const finish = FINISH_OF[symbolOf('mode_landing', params.mode_landing)];
  const supportedActions = new Set(categoryModel.getActions({ type: task.uav_type }).map((a) => a.name));

  const items = [];
  const wpSeqs = [];
  // ArduPilot overwrites seq 0 with the home: whatever goes there never flies.
  if (isArdupilot) items.push({ command: MAV_CMD_MISSION.NAV_WAYPOINT, params: [0, 0, 0, 0], ...at(waypoints[0].pos) });

  waypoints.forEach((wp, i) => {
    const action = wp.action ?? {};
    for (const name of Object.keys(action)) {
      if (!supportedActions.has(name)) logger.warn(`MAVLink mission: action '${name}' not supported by ${task.uav_type}, skipping`);
    }
    const has = (name) => supportedActions.has(name) && action[name] != null;
    // Yaw at this waypoint: the action wins over the waypoint field; only in WAYPOINT_CUSTOM
    // does wp.yaw count (in AUTO the autopilot faces the next waypoint).
    const yaw = has('yaw') ? Number(action.yaw) : yawMode === 'HEADING_MODE_WAYPOINT_CUSTOM' && wp.yaw != null ? Number(wp.yaw) : null;
    const hold = has('stay') ? Number(action.stay) : 0;

    if (wp.speed != null && i > 0) items.push(changeSpeed(Number(wp.speed)));

    if (i === 0 && wp.type === 'takeoff') {
      wpSeqs.push(items.length);
      items.push({ command: MAV_CMD_MISSION.NAV_TAKEOFF, params: [0, 0, 0, NaN], ...at(wp.pos) });
      // The rest of the route flies at idle_vel unless a waypoint says otherwise.
      if (params.idle_vel != null) items.push(changeSpeed(Number(params.idle_vel)));
    } else {
      if (i === 0) {
        // A route that does not start with a takeoff waypoint still has to leave the ground:
        // take off over the first waypoint (an airborne vehicle skips it).
        items.push({ command: MAV_CMD_MISSION.NAV_TAKEOFF, params: [0, 0, 0, NaN], ...at(wp.pos) });
        if (params.idle_vel != null) items.push(changeSpeed(Number(params.idle_vel)));
      }
      wpSeqs.push(items.length);
      const yawParam = !isArdupilot && yaw != null ? yaw : NaN;
      items.push({ command: MAV_CMD_MISSION.NAV_WAYPOINT, params: [hold, 0, 0, yawParam], ...at(wp.pos) });
    }
    if (isArdupilot && yaw != null) items.push(conditionYaw(yaw));

    const pitch = has('gimbal') ? Number(action.gimbal) : wp.gimbal != null ? Number(wp.gimbal) : null;
    if (pitch != null) items.push(gimbalPitch(pitch, isArdupilot));
    if (has('photo')) items.push(photo());
    if (has('video_start')) items.push(videoStart());
    if (has('video_stop')) items.push(videoStop());
  });

  const last = waypoints.at(-1);
  if (finish === MISSION_FINISH.HOME) items.push({ command: MAV_CMD_MISSION.NAV_RETURN_TO_LAUNCH, params: [0, 0, 0, 0], ...notAt() });
  if (finish === MISSION_FINISH.LAND) {
    items.push({ command: MAV_CMD_MISSION.NAV_LAND, params: [0, 0, 0, NaN], ...at([last.pos[0], last.pos[1], 0]) });
  }

  // PX4 keeps the mission cursor between missions: an upload with no item flagged `current`
  // resumes at the index the previous run ended on (navigator/mission_base.cpp clamps the old
  // seq), and a mission that ended there is "finished" the moment it starts.
  if (!isArdupilot) items[0].current = true;

  return { kind: 'mission', items, wpSeqs, finish };
}
