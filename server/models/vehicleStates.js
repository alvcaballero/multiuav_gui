import { ARM_STATE, FLIGHT_PHASE, FLIGHT_MODE, COMMAND_RESULT } from '../config/status.js';

// A firmware's own state codes -> the GCS vocabulary (ARM_STATE, FLIGHT_PHASE, FLIGHT_MODE in
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
