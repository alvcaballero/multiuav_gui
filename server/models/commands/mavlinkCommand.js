// MAVLink command builder — command type (config/commandTypes.js) → MAV_CMD over MAVLink, per
// firmware dialect. The sibling of vehicleCommand.js (PX4 over ROS 2 / uORB).
//
// Pure: receives the device's current position and returns DESCRIPTORS, not node-mavlink objects:
//   { kind: 'long', command, params[7], targetComponent? }
//   { kind: 'int',  command, frame, params[4], lat, lon, alt, targetComponent? }
// MavlinkServer turns them into COMMAND_LONG / COMMAND_INT (mavlinkMessages.js, the only place
// that knows node-mavlink's `_paramN`) and sends them in order.
//
// The dialect is the endpoint's messageType: the category says which firmware the device runs,
// and MavlinkServer refuses to send when the vehicle's heartbeat reports another autopilot.
//
// Verified in SITL (PX4 1.18.0, ArduCopter 4.7.2) and in their sources, MAVLINK_PLAN.md › F0:
//   - Anything with a lat/lon goes as COMMAND_INT: COMMAND_LONG floats quantise the position (0.18 m
//     here) and ArduPilot answers UNSUPPORTED to DO_REPOSITION over COMMAND_LONG.
//   - NAV_TAKEOFF altitude: PX4 reads it as AMSL and IGNORES the COMMAND_INT frame
//     (mavlink_receiver.cpp copies z → param7; VehicleCommand has no frame). ArduCopter only accepts
//     frame GLOBAL_RELATIVE_ALT (anything else is DENIED), lat/lon ignored, and only in GUIDED.
//   - DO_REPOSITION yaw: PX4 takes param4 in RADIANS over MAVLink too (navigator copies it as-is,
//     unlike the spec's degrees). ArduCopter ignores it: the turn is a CONDITION_YAW after it.
//   - Takeoff does not arm, as over ROS: Arm is its own command.

import { CommandType } from '../../config/commandTypes.js';
import { PublisherKey, ServiceKey } from '../../config/deviceMsgCatalog.js';
import { MAVLINK_MISSION } from '../mission/missionSymbols.js';
import { relativeTarget } from './vehicleCommand.js';

export const MAVLINK_COMMAND = Object.freeze({
  PX4: 'mavlink/PX4_COMMAND',
  ARDUPILOT: 'mavlink/ARDUPILOT_COMMAND',
});

// MAV_AUTOPILOT a dialect is for: MavlinkServer checks it against the vehicle's heartbeat.
// The mission upload (configure_mission, missionEncodeMavlink.js) has its own dialect msgType.
export const DIALECT_AUTOPILOT = Object.freeze({
  [MAVLINK_COMMAND.PX4]: 12, // MAV_AUTOPILOT_PX4
  [MAVLINK_COMMAND.ARDUPILOT]: 3, // MAV_AUTOPILOT_ARDUPILOTMEGA
  [MAVLINK_MISSION.PX4]: 12,
  [MAVLINK_MISSION.ARDUPILOT]: 3,
});

export const MAV_CMD = Object.freeze({
  NAV_RETURN_TO_LAUNCH: 20,
  NAV_LAND: 21,
  NAV_TAKEOFF: 22,
  CONDITION_YAW: 115,
  DO_SET_MODE: 176,
  MISSION_START: 300,
  DO_REPOSITION: 192,
  COMPONENT_ARM_DISARM: 400,
  DO_GIMBAL_MANAGER_PITCHYAW: 1000,
  DO_GIMBAL_MANAGER_CONFIGURE: 1001,
});

const MAV_FRAME_GLOBAL = 0; // AMSL
const MAV_FRAME_GLOBAL_RELATIVE_ALT = 3;
const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;
const PX4_MAIN_MODE_AUTO = 4;
const PX4_SUB_MODE_AUTO_LOITER = 3;
const PX4_SUB_MODE_AUTO_MISSION = 4;
const COPTER_MODE_AUTO = 3;
const COPTER_MODE_GUIDED = 4;
// Hold on ArduCopter is BRAKE, never LOITER: LOITER climbs/descends with the pilot's throttle
// stick (mode_loiter.cpp: get_pilot_desired_climb_rate), and with no RC or the stick low it
// DESCENDS — in SITL a paused mission went from 12 m to the ground. BRAKE is an autopilot mode
// that holds position and altitude and ignores the pilot (its timeout to LOITER is Solo-only).
const COPTER_MODE_BRAKE = 17;
const FORCE_DISARM_MAGIC = 21196; // what QGC's emergency stop sends; both firmwares honour it
const REPOSITION_DEFAULT_SPEED = -1;
const REPOSITION_CHANGE_MODE = 1; // MAV_DO_REPOSITION_FLAGS_CHANGE_MODE (ArduCopter: switch to GUIDED)
const MAV_COMP_ID_ALL = 0;
const GCS_SYSTEM_ID = 255;
const GCS_COMPONENT_ID = 190;
const LEAVE_UNCHANGED = -1;
const GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16;

const long = (command, params = [], targetComponent) => ({ kind: 'long', command, params, ...(targetComponent != null && { targetComponent }) });
const int = (command, { frame, params = [], lat, lon, alt }) => ({ kind: 'int', command, frame, params, lat, lon, alt });

const RAD_TO_DEG = 180 / Math.PI;
const heading360 = (rad) => (((rad * RAD_TO_DEG) % 360) + 360) % 360;

function requirePosition(position, type) {
  if (position?.latitude == null || position?.longitude == null || position?.altitude == null) {
    throw new Error(`${type} needs the current position of the device and none is known`);
  }
}

const arm = (on) => long(MAV_CMD.COMPONENT_ARM_DISARM, [on ? 1 : 0]);
const killMotors = () => long(MAV_CMD.COMPONENT_ARM_DISARM, [0, FORCE_DISARM_MAGIC]);
const returnHome = () => long(MAV_CMD.NAV_RETURN_TO_LAUNCH);

const px4Mode = (sub) => long(MAV_CMD.DO_SET_MODE, [MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MAIN_MODE_AUTO, sub]);
const copterMode = (mode) => long(MAV_CMD.DO_SET_MODE, [MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode]);

function px4Command(type, attributes, position) {
  switch (type) {
    // Mission (uploaded by LoadMission). Start = arm + AUTO.MISSION; its first item takes off.
    // Pause and Stop hold where it is (Stop = Hold, decided by the user); Resume goes back to
    // AUTO.MISSION, which continues from the current item.
    case CommandType.COMMAND_MISSION:
      return [arm(true), px4Mode(PX4_SUB_MODE_AUTO_MISSION)];
    case CommandType.PAUSE_MISSION:
    case CommandType.STOP_MISSION:
      return px4Mode(PX4_SUB_MODE_AUTO_LOITER);
    case CommandType.RESUME_MISSION:
      return px4Mode(PX4_SUB_MODE_AUTO_MISSION);
    case CommandType.TAKEOFF:
      requirePosition(position, type);
      // lat/lon/yaw NaN: take off where it is (navigator sets the yaw itself, NAN in the triplet).
      return long(MAV_CMD.NAV_TAKEOFF, [0, 0, 0, NaN, NaN, NaN, position.altitude + attributes.altitude]);
    case CommandType.LAND:
      return long(MAV_CMD.NAV_LAND, [0, 0, 0, NaN, NaN, NaN, NaN]);
    case CommandType.RETURN_HOME:
      return returnHome();
    case CommandType.HOLD:
      return long(MAV_CMD.DO_SET_MODE, [MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_LOITER]);
    case CommandType.ARM:
      return arm(true);
    case CommandType.DISARM:
      return arm(false);
    case CommandType.KILL_MOTORS:
      return killMotors();
    case CommandType.MOVE_RELATIVE: {
      requirePosition(position, type);
      const t = relativeTarget(position, attributes);
      return int(MAV_CMD.DO_REPOSITION, {
        frame: MAV_FRAME_GLOBAL,
        params: [REPOSITION_DEFAULT_SPEED, REPOSITION_CHANGE_MODE, 0, t.yaw], // yaw in rad (see header)
        lat: t.latitude,
        lon: t.longitude,
        alt: t.altitude,
      });
    }
    case CommandType.GIMBAL:
      // The gimbal manager only obeys PITCHYAW from its primary control: take it first, every time
      // (a mission action or the RC can take it). Same reasoning as vehicleCommand.js.
      return [
        long(MAV_CMD.DO_GIMBAL_MANAGER_CONFIGURE, [GCS_SYSTEM_ID, GCS_COMPONENT_ID, LEAVE_UNCHANGED, LEAVE_UNCHANGED, 0, 0, 0], MAV_COMP_ID_ALL),
        // Rates NaN = "angle only": with 0 PX4 ACKs ACCEPTED but the gimbal does not move.
        long(MAV_CMD.DO_GIMBAL_MANAGER_PITCHYAW, [attributes.pitch, attributes.yaw, NaN, NaN, GIMBAL_MANAGER_FLAGS_YAW_LOCK, 0, 0], MAV_COMP_ID_ALL),
      ];
    default:
      throw new Error(`'${type}' is not a PX4 MAVLink command`);
  }
}

function ardupilotCommand(type, attributes, position) {
  const guided = copterMode(COPTER_MODE_GUIDED);
  switch (type) {
    // ArduCopter does not arm in AUTO: arm in GUIDED, then MISSION_START switches to AUTO and
    // runs from the first item (the takeoff). Resume = AUTO, which continues the current item.
    case CommandType.COMMAND_MISSION:
      return [guided, arm(true), long(MAV_CMD.MISSION_START, [0, 0])];
    case CommandType.PAUSE_MISSION:
    case CommandType.STOP_MISSION:
      return copterMode(COPTER_MODE_BRAKE);
    case CommandType.RESUME_MISSION:
      return copterMode(COPTER_MODE_AUTO);
    case CommandType.TAKEOFF:
      // GUIDED first: ArduCopter takes off only in a guided mode. z is relative (frame enforced by
      // ArduCopter); lat/lon are ignored, so no position is needed.
      return [
        guided,
        int(MAV_CMD.NAV_TAKEOFF, {
          frame: MAV_FRAME_GLOBAL_RELATIVE_ALT,
          params: [0, 0, 0, NaN],
          lat: position?.latitude ?? 0,
          lon: position?.longitude ?? 0,
          alt: attributes.altitude,
        }),
      ];
    case CommandType.LAND:
      return long(MAV_CMD.NAV_LAND, [0, 0, 0, NaN, 0, 0, 0]);
    case CommandType.RETURN_HOME:
      return returnHome();
    case CommandType.HOLD:
      return copterMode(COPTER_MODE_BRAKE);
    case CommandType.ARM:
      return arm(true);
    case CommandType.DISARM:
      return arm(false);
    case CommandType.KILL_MOTORS:
      return killMotors();
    case CommandType.MOVE_RELATIVE: {
      requirePosition(position, type);
      const t = relativeTarget(position, attributes);
      const reposition = int(MAV_CMD.DO_REPOSITION, {
        frame: MAV_FRAME_GLOBAL,
        params: [REPOSITION_DEFAULT_SPEED, REPOSITION_CHANGE_MODE, 0, NaN],
        lat: t.latitude,
        lon: t.longitude,
        alt: t.altitude,
      });
      if (!attributes.dyaw) return reposition;
      // DO_REPOSITION ignores the yaw on ArduCopter: turn with CONDITION_YAW (absolute heading, deg).
      return [reposition, long(MAV_CMD.CONDITION_YAW, [heading360(t.yaw), 0, 0, 0])];
    }
    default:
      throw new Error(`'${type}' is not an ArduPilot MAVLink command`);
  }
}

/**
 * @param {object} args
 * @param {string} args.dialect      MAVLINK_COMMAND.* (the endpoint's messageType)
 * @param {string} args.type         CommandType
 * @param {object} [args.attributes] validated and defaulted by command_schema
 * @param {object} [args.position]   current position (latitude, longitude, altitude AMSL, course)
 * @returns {object|object[]} one descriptor, or several to send in order
 */
export function buildMavlinkCommand({ dialect, type, attributes = {}, position }) {
  if (dialect === MAVLINK_COMMAND.PX4) return px4Command(type, attributes, position);
  if (dialect === MAVLINK_COMMAND.ARDUPILOT) return ardupilotCommand(type, attributes, position);
  throw new Error(`unknown MAVLink dialect '${dialect}'`);
}

// Every endpoint of a mavlink category shares its dialect msgType, so a command is registered by
// (endpoint key, msgType): by msgType alone, Takeoff would match command_mission too.
const FLIGHT = PublisherKey.VEHICLE_COMMAND;
const COMMON_COMMANDS = {
  [CommandType.TAKEOFF]: FLIGHT,
  [CommandType.LAND]: FLIGHT,
  [CommandType.RETURN_HOME]: FLIGHT,
  [CommandType.HOLD]: FLIGHT,
  [CommandType.ARM]: FLIGHT,
  [CommandType.DISARM]: FLIGHT,
  [CommandType.KILL_MOTORS]: FLIGHT,
  [CommandType.MOVE_RELATIVE]: FLIGHT,
  [CommandType.COMMAND_MISSION]: ServiceKey.COMMAND_MISSION,
  [CommandType.PAUSE_MISSION]: ServiceKey.PAUSE_MISSION,
  [CommandType.RESUME_MISSION]: ServiceKey.RESUME_MISSION,
  [CommandType.STOP_MISSION]: ServiceKey.STOP_MISSION,
};

/** command type → the endpoint key that implements it, per dialect (what commandRegistry registers). */
export const MAVLINK_COMMAND_TYPES = Object.freeze({
  [MAVLINK_COMMAND.PX4]: { ...COMMON_COMMANDS, [CommandType.GIMBAL]: FLIGHT },
  // Gimbal is not offered: not verified (the ArduCopter SITL has no mount).
  [MAVLINK_COMMAND.ARDUPILOT]: { ...COMMON_COMMANDS },
});
