import {
  ARM_STATE,
  FLIGHT_PHASE,
  FLIGHT_MODE,
  COMMAND_RESULT,
  VEHICLE_TYPE,
  VTOL_STATE,
  CONTROL_SOURCE,
  MISSION_EXEC_STATE,
  MISSION_EVENT,
  MISSION_STOP_REASON,
} from '../config/status.js';

// A firmware's own state codes -> the GCS vocabulary (ARM_STATE, FLIGHT_PHASE, FLIGHT_MODE, VTOL_STATE in
// config/status.js). One adapter per message that carries them, used by that message's decoder.
// Not under ros/: the same codes arrive over rosbridge, MQTT and robofleet, and each transport's
// decoder (msgDecode.js, fbDecode.js) calls the same adapter, as models/msgEncode.js does on the way out.
// An unknown code is never an error: arm and phase fall back to UNKNOWN, a mode to OTHER, and
// the decoder keeps the raw code next to the canonical value.

// ─── PX4 px4_msgs/msg/VehicleStatus ────────────────────────────────────────────

const PX4_ARM = { 1: ARM_STATE.DISARMED, 2: ARM_STATE.ARMED };

// nav_state: the modes the GCS does not tell apart are grouped; Free*/Max are not real modes.
const PX4_MODE = {
  0: FLIGHT_MODE.MANUAL,
  1: FLIGHT_MODE.ASSISTED, // altitude
  2: FLIGHT_MODE.ASSISTED, // position
  3: FLIGHT_MODE.MISSION,
  4: FLIGHT_MODE.HOLD, // loiter
  5: FLIGHT_MODE.RTL,
  6: FLIGHT_MODE.ASSISTED, // position slow
  10: FLIGHT_MODE.MANUAL, // acro
  12: FLIGHT_MODE.LAND, // descend
  13: FLIGHT_MODE.EMERGENCY, // termination
  14: FLIGHT_MODE.OFFBOARD,
  15: FLIGHT_MODE.MANUAL, // stabilized
  17: FLIGHT_MODE.TAKEOFF,
  18: FLIGHT_MODE.LAND,
  19: FLIGHT_MODE.AUTO_OTHER, // follow target
  20: FLIGHT_MODE.LAND, // precision land
  21: FLIGHT_MODE.AUTO_OTHER, // orbit
  22: FLIGHT_MODE.TAKEOFF, // VTOL takeoff
};

export const px4ArmState = (code) => PX4_ARM[code] ?? ARM_STATE.UNKNOWN;
export const px4FlightMode = (code) => PX4_MODE[code] ?? FLIGHT_MODE.OTHER;

// ─── PX4 muav_state_machine/UAVState.landed_state (MAV_LANDED_STATE) ───────────

const PX4_PHASE = [
  FLIGHT_PHASE.UNKNOWN,
  FLIGHT_PHASE.ON_GROUND,
  FLIGHT_PHASE.IN_AIR,
  FLIGHT_PHASE.TAKEOFF,
  FLIGHT_PHASE.LANDING,
];

export const px4FlightPhase = (code) => PX4_PHASE[code] ?? FLIGHT_PHASE.UNKNOWN;

// ─── PX4 px4_msgs/msg/VehicleCommandAck.result (MAV_RESULT) ────────────────────

const PX4_COMMAND_RESULT = [
  COMMAND_RESULT.ACCEPTED,
  COMMAND_RESULT.REJECTED,
  COMMAND_RESULT.DENIED,
  COMMAND_RESULT.UNSUPPORTED,
  COMMAND_RESULT.FAILED,
  COMMAND_RESULT.IN_PROGRESS,
  COMMAND_RESULT.CANCELLED,
];

export const px4CommandResult = (code) => PX4_COMMAND_RESULT[code] ?? COMMAND_RESULT.UNKNOWN;

// ─── MAVLink HEARTBEAT (PX4 and ArduPilot over MAVLink) ────────────────────────
// PX4's landed_state and command result above ARE the MAVLink enums (MAV_LANDED_STATE,
// MAV_RESULT): EXTENDED_SYS_STATE and COMMAND_ACK reuse those adapters for both firmwares.
// The mode is not: custom_mode means something else per autopilot, which the heartbeat itself
// says (`autopilot`), and for ArduPilot per vehicle type too.

const MAV_MODE_FLAG_SAFETY_ARMED = 128;
const MAV_AUTOPILOT_ARDUPILOTMEGA = 3;
const MAV_AUTOPILOT_PX4 = 12;
// MAV_TYPE of the multirotors ArduCopter flies (quad, coaxial, heli, hexa, octo, tri, dodeca,
// deca). Plane and Rover number their modes differently: they fall to OTHER.
const COPTER_TYPES = new Set([2, 3, 4, 13, 14, 15, 29, 35]);

// MAV_TYPE (HEARTBEAT.type) → the GCS vehicle type. 1 FIXED_WING; 10 GROUND_ROVER; 11 SURFACE_BOAT;
// 19..25 the VTOL family (tailsitters, tiltrotor, fixed rotor, tiltwing). Anything else (a submarine,
// a gimbal, an antenna tracker) says nothing about it: null.
const MAV_VTOL_TYPES = new Set([19, 20, 21, 22, 23, 24, 25]);

export function vehicleTypeOfMavType(type) {
  if (COPTER_TYPES.has(type)) return VEHICLE_TYPE.MULTIROTOR;
  if (MAV_VTOL_TYPES.has(type)) return VEHICLE_TYPE.VTOL;
  if (type === 1) return VEHICLE_TYPE.FIXED_WING;
  if (type === 10) return VEHICLE_TYPE.UGV;
  if (type === 11) return VEHICLE_TYPE.USV;
  return null;
}

// EXTENDED_SYS_STATE.vtol_state (MAV_VTOL_STATE). 0 UNDEFINED is what a vehicle that is not a VTOL
// sends: null, so the decoder writes nothing (a multirotor has no VTOL axis, not an UNKNOWN one).
const MAV_VTOL_STATE = [null, VTOL_STATE.TRANSITION_TO_FW, VTOL_STATE.TRANSITION_TO_MC, VTOL_STATE.MC, VTOL_STATE.FW];

export const mavlinkVtolState = (code) =>
  code == null || code === 0 ? null : (MAV_VTOL_STATE[code] ?? VTOL_STATE.UNKNOWN);

export const mavlinkArmState = (baseMode) =>
  baseMode == null ? ARM_STATE.UNKNOWN : baseMode & MAV_MODE_FLAG_SAFETY_ARMED ? ARM_STATE.ARMED : ARM_STATE.DISARMED;

// PX4 custom_mode = (sub_mode << 24) | (main_mode << 16). Both enums of
// src/modules/commander/px4_custom_mode.h count implicitly from 1: check there, not from memory.
const PX4_MAIN_MODE = {
  1: FLIGHT_MODE.MANUAL,
  2: FLIGHT_MODE.ASSISTED, // altitude
  3: FLIGHT_MODE.ASSISTED, // position
  5: FLIGHT_MODE.MANUAL, // acro
  6: FLIGHT_MODE.OFFBOARD,
  7: FLIGHT_MODE.MANUAL, // stabilized
  8: FLIGHT_MODE.MANUAL, // rattitude (legacy)
  9: FLIGHT_MODE.ASSISTED, // simple
  10: FLIGHT_MODE.EMERGENCY, // termination
  11: FLIGHT_MODE.ASSISTED, // altitude cruise
};
const PX4_MAIN_MODE_AUTO = 4;
const PX4_AUTO_SUB_MODE = {
  2: FLIGHT_MODE.TAKEOFF,
  3: FLIGHT_MODE.HOLD, // loiter
  4: FLIGHT_MODE.MISSION,
  5: FLIGHT_MODE.RTL,
  6: FLIGHT_MODE.LAND,
  8: FLIGHT_MODE.RTL, // RTGS: return to the ground station
  9: FLIGHT_MODE.AUTO_OTHER, // follow target
  10: FLIGHT_MODE.LAND, // precision land
  11: FLIGHT_MODE.TAKEOFF, // VTOL takeoff
  12: FLIGHT_MODE.LAND, // descend
};

export function px4CustomMode(customMode) {
  const main = (customMode >>> 16) & 0xff;
  const sub = (customMode >>> 24) & 0xff;
  if (main === PX4_MAIN_MODE_AUTO) return PX4_AUTO_SUB_MODE[sub] ?? FLIGHT_MODE.OTHER;
  return PX4_MAIN_MODE[main] ?? FLIGHT_MODE.OTHER;
}

// ArduCopter custom_mode = its mode number (ArduCopter/mode.h). GUIDED is "the GCS or a companion
// sends the targets": OFFBOARD, as DJI's NAVI_SDK_CTRL. Takeoff and goto fly in GUIDED.
const COPTER_MODE = {
  0: FLIGHT_MODE.MANUAL, // stabilize
  1: FLIGHT_MODE.MANUAL, // acro
  2: FLIGHT_MODE.ASSISTED, // alt hold
  3: FLIGHT_MODE.MISSION, // auto
  4: FLIGHT_MODE.OFFBOARD, // guided
  5: FLIGHT_MODE.HOLD, // loiter
  6: FLIGHT_MODE.RTL,
  7: FLIGHT_MODE.AUTO_OTHER, // circle
  9: FLIGHT_MODE.LAND,
  11: FLIGHT_MODE.ASSISTED, // drift
  13: FLIGHT_MODE.ASSISTED, // sport
  14: FLIGHT_MODE.MANUAL, // flip
  15: FLIGHT_MODE.AUTO_OTHER, // autotune
  16: FLIGHT_MODE.ASSISTED, // poshold
  17: FLIGHT_MODE.HOLD, // brake
  18: FLIGHT_MODE.AUTO_OTHER, // throw
  19: FLIGHT_MODE.AUTO_OTHER, // avoid ADS-B
  20: FLIGHT_MODE.OFFBOARD, // guided no GPS
  21: FLIGHT_MODE.RTL, // smart RTL
  22: FLIGHT_MODE.ASSISTED, // flow hold
  23: FLIGHT_MODE.AUTO_OTHER, // follow
  24: FLIGHT_MODE.AUTO_OTHER, // zigzag
  26: FLIGHT_MODE.EMERGENCY, // autorotate
  27: FLIGHT_MODE.RTL, // auto RTL (mission's DO_LAND_START)
  28: FLIGHT_MODE.MANUAL, // turtle (flip a crashed copter back)
};

export const ardupilotCopterMode = (customMode) => COPTER_MODE[customMode] ?? FLIGHT_MODE.OTHER;

/** The mode of a MAVLink HEARTBEAT, from its own autopilot and vehicle type. */
export function mavlinkFlightMode({ autopilot, type, customMode }) {
  if (autopilot === MAV_AUTOPILOT_PX4) return px4CustomMode(customMode);
  if (autopilot === MAV_AUTOPILOT_ARDUPILOTMEGA && COPTER_TYPES.has(type)) return ardupilotCopterMode(customMode);
  return FLIGHT_MODE.OTHER;
}

// ─── DJI flight_status (std_msgs/UInt8: STOPED=0, ON_GROUND=1, ON_AIR=2) ───────
// One code carries two axes: STOPED is "motors stopped", which is a disarmed vehicle on the ground.

const DJI_FLIGHT_STATUS = [
  { armState: ARM_STATE.DISARMED, flightPhase: FLIGHT_PHASE.ON_GROUND },
  { armState: ARM_STATE.ARMED, flightPhase: FLIGHT_PHASE.ON_GROUND },
  { armState: ARM_STATE.ARMED, flightPhase: FLIGHT_PHASE.IN_AIR },
];

/** @returns {{armState: string, flightPhase: string}} */
export const djiFlightStatus = (code) =>
  DJI_FLIGHT_STATUS[code] ?? { armState: ARM_STATE.UNKNOWN, flightPhase: FLIGHT_PHASE.UNKNOWN };

// ─── DJI display_mode (dji_sdk, std_msgs/UInt8: DISPLAY_MODE_*) ────────────────
// 17 (NAVI_SDK_CTRL) is "an onboard computer flies it" and, as far as I know, also what a waypoint
// mission shows on the OSDK: both land on OFFBOARD, so a DJI mission is never MISSION.
// Reserved codes and the ones with no clear meaning (SEARCH 40, ENGINE_START 41) are OTHER.

const DJI_MODE = {
  0: FLIGHT_MODE.MANUAL, // MANUAL_CTRL
  1: FLIGHT_MODE.ASSISTED, // ATTITUDE
  6: FLIGHT_MODE.ASSISTED, // P_GPS
  9: FLIGHT_MODE.AUTO_OTHER, // HOTPOINT
  10: FLIGHT_MODE.TAKEOFF, // ASSISTED_TAKEOFF
  11: FLIGHT_MODE.TAKEOFF, // AUTO_TAKEOFF
  12: FLIGHT_MODE.LAND, // AUTO_LANDING
  15: FLIGHT_MODE.RTL, // NAVI_GO_HOME
  17: FLIGHT_MODE.OFFBOARD, // NAVI_SDK_CTRL
  33: FLIGHT_MODE.LAND, // FORCE_AUTO_LANDING
};

export const djiFlightMode = (code) => DJI_MODE[code] ?? FLIGHT_MODE.OTHER;

// ─── DJI control_mode (psdk_interfaces/msg/ControlMode) ────────────────────────
// device_mode: who the authority belongs to (0 RC, 1 MSDK, 4 PSDK); control_auth: 1 = that device HOLDS it.

const DJI_CONTROL_DEVICE = { 0: CONTROL_SOURCE.PILOT, 1: CONTROL_SOURCE.MOBILE_APP, 4: CONTROL_SOURCE.ONBOARD };

/** @returns {{controlSource: string, controlAuthority: boolean}} */
export const djiControlMode = (deviceMode, controlAuth) => ({
  controlSource: DJI_CONTROL_DEVICE[deviceMode] ?? CONTROL_SOURCE.UNKNOWN,
  controlAuthority: controlAuth === 1,
});

// ─── DJI WaypointV2 state push (dji_waypoint_v2_type.h) ────────────────────────
// The comment in WaypointV2MissionStatePush.msg (0-2 only) is outdated: these are the SDK's values.

const DJI_WPV2_STATE = [
  MISSION_EXEC_STATE.IDLE, // GROUND_STATION_NOT_START
  MISSION_EXEC_STATE.PREPARED, // MISSION_PREPARED
  MISSION_EXEC_STATE.STARTING, // ENTER_MISSION
  MISSION_EXEC_STATE.RUNNING, // EXECUTE_FLYING_ROUTE_MISSION
  MISSION_EXEC_STATE.PAUSED, // PAUSED_STATE
  MISSION_EXEC_STATE.RESUMING, // ENTER_MISSION_AFTER_ENDING_PAUSE
  MISSION_EXEC_STATE.EXITING, // EXIT_MISSION
];

export const djiWaypointV2State = (code) => DJI_WPV2_STATE[code] ?? MISSION_EXEC_STATE.UNKNOWN;

// ─── DJI WaypointV2 event push (psdk_ros2 waypoint_flying.cpp) ─────────────────
// 0x30 (action switch) arrives over and over and says nothing to an operator: OTHER, dropped by the reducer.

const DJI_WPV2_EVENT = {
  0x01: MISSION_EVENT.INTERRUPTED,
  0x02: MISSION_EVENT.RESUMED,
  0x03: MISSION_EVENT.STOPPED,
  0x10: MISSION_EVENT.WP_REACHED,
  0x11: MISSION_EVENT.FINISHED,
  0x12: MISSION_EVENT.AVOIDING_OBSTACLE,
};

export const djiWaypointV2Event = (code) => DJI_WPV2_EVENT[code] ?? MISSION_EVENT.OTHER;

// finish_reason of the stop event (0x03).
const DJI_WPV2_STOP_REASON = {
  0x00: MISSION_STOP_REASON.COMPLETED,
  0x10: MISSION_STOP_REASON.USER,
  0x20: MISSION_STOP_REASON.MOTORS_STARTING,
  0x21: MISSION_STOP_REASON.TAKEOFF_TIMEOUT,
  0x22: MISSION_STOP_REASON.TAKEOFF_ALTITUDE,
  0x23: MISSION_STOP_REASON.TAKEOFF_ABNORMAL,
  0x24: MISSION_STOP_REASON.PREEMPTED,
  0x26: MISSION_STOP_REASON.PATH_PLANNING,
};

export const djiWaypointV2StopReason = (code) => DJI_WPV2_STOP_REASON[code] ?? MISSION_STOP_REASON.UNKNOWN;

// ─── CATEC state_machine (std_msgs/String) ─────────────────────────────────────
// Free text from the CATEC state machine. Only the phase is known: it says nothing about arming.

const CATEC_PHASE = {
  STOPED: FLIGHT_PHASE.ON_GROUND,
  'ON GROUND': FLIGHT_PHASE.ON_GROUND,
  TAKEOFF: FLIGHT_PHASE.TAKEOFF,
  HOVER: FLIGHT_PHASE.IN_AIR,
  'IN AIR': FLIGHT_PHASE.IN_AIR,
  LANDING: FLIGHT_PHASE.LANDING,
};

export const catecFlightState = (text) => ({
  flightPhase: CATEC_PHASE[String(text ?? '').toUpperCase()] ?? FLIGHT_PHASE.UNKNOWN,
});

// ─── muav_state_machine/UAVState.landed_state ──────────────────────────────────
// The code means whatever the airframe it comes from means; `airframe_type` is data in the
// message, not a category name.

/** @returns {{armState?: string, flightPhase: string}} */
export function landedStateOf(airframe, code) {
  if (airframe === 'dji') return djiFlightStatus(code);
  if (airframe === 'catec') return catecFlightState(code);
  return { flightPhase: px4FlightPhase(code) };
}
