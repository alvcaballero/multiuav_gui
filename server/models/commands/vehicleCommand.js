// PX4 VehicleCommand builder — command type (config/commandTypes.js) → px4_msgs/msg/VehicleCommand.
//
// Pure: receives the device's current position and returns the full wire message, so it
// is testable without ROS. commandsModel publishes the result on the device's
// `VehicleCommand` publisher (/fmu/in/vehicle_command).
//
// Notes that are easy to get wrong:
//   - The message goes straight to uORB, so angles are RADIANS (navigator writes
//     param4 into position_setpoint.yaw as-is), unlike the MAVLink spec's degrees.
//   - NaN reaches rosbridge as the bare `NaN` literal (rosConnection's transport), but MQTT's
//     JSON would turn it into null. Positions/angles PX4 would read as NaN are filled with the
//     current value; only the gimbal rates go as NaN (a finite 0 doesn't move the gimbal).
//   - target_system is the vehicle's MAV_SYS_ID, different per drone (multi-vehicle SITL:
//     instance + 1). It comes from telemetry (vehicle_status.system_id →
//     position.attributes.systemId). Unknown → the command is REJECTED, never sent as 0
//     (broadcast): a KillMotors would reach every vehicle on the link.
//     target_component 1 = MAV_COMP_ID_AUTOPILOT1; the gimbal manager takes 0 (any component).
//   - KillMotors is a FORCED DISARM (param2 = 21196, what QGC's emergency stop sends),
//     not DO_FLIGHTTERMINATION: PX4 ships with CBRK_FLIGHTTERM set, which ignores it.

import { CommandType } from '../../config/commandTypes.js';
import { offsetLatLon } from '../../common/geo.js';

export const VEHICLE_CMD = Object.freeze({
  VEHICLE_CMD_NAV_RETURN_TO_LAUNCH: 20,
  VEHICLE_CMD_NAV_LAND: 21,
  VEHICLE_CMD_NAV_TAKEOFF: 22,
  VEHICLE_CMD_DO_SET_MODE: 176,
  VEHICLE_CMD_DO_REPOSITION: 192,
  VEHICLE_CMD_COMPONENT_ARM_DISARM: 400,
  VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW: 1000,
  VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE: 1001,
});

const FORCE_DISARM_MAGIC = 21196;
// DO_SET_MODE params for Hold: custom mode enabled, PX4 main mode AUTO, sub mode LOITER.
const MODE_CUSTOM = 1;
const PX4_MAIN_MODE_AUTO = 4;
const PX4_SUB_MODE_AUTO_LOITER = 3;
// DO_REPOSITION param2 bit 0: switch to the mode that flies the reposition.
const REPOSITION_CHANGE_MODE = 1;
const MAV_COMP_ID_AUTOPILOT1 = 1;
const MAV_COMP_ID_ALL = 0;
// Who sends every VehicleCommand: the standard MAVLink identity of a ground station.
const GCS_SYSTEM_ID = 255;
const GCS_COMPONENT_ID = 190;
// DO_GIMBAL_MANAGER_CONFIGURE: -1 leaves the secondary control unchanged; gimbal device 0 = all.
const LEAVE_UNCHANGED = -1;
const ALL_GIMBAL_DEVICES = 0;
// DO_GIMBAL_MANAGER_PITCHYAW param5 (GIMBAL_MANAGER_FLAGS): yaw relative to North, not to the
// vehicle heading. Absolute angles, as command_schema's Gimbal contract says. PX4's gimbal module
// only reads YAW_LOCK (input_mavlink.cpp): YAW_IN_EARTH_FRAME (64) is ignored there.
const GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16;

const DEG_TO_RAD = Math.PI / 180;
const wrapPi = (rad) => Math.atan2(Math.sin(rad), Math.cos(rad));

function requirePosition(position, type) {
  if (position?.latitude == null || position?.longitude == null || position?.altitude == null) {
    throw new Error(`${type} needs the current position of the device and none is known`);
  }
}

function systemIdOf(position, type) {
  const systemId = position?.attributes?.systemId;
  if (systemId == null) {
    throw new Error(`${type} needs the vehicle's system id (vehicle_status telemetry) and none is known`);
  }
  return systemId;
}

function message(command, { system, component }, params = {}) {
  return {
    timestamp: 0,
    param1: params.param1 ?? 0,
    param2: params.param2 ?? 0,
    param3: params.param3 ?? 0,
    param4: params.param4 ?? 0,
    param5: params.param5 ?? 0,
    param6: params.param6 ?? 0,
    param7: params.param7 ?? 0,
    command,
    target_system: system,
    target_component: component,
    source_system: GCS_SYSTEM_ID,
    source_component: GCS_COMPONENT_ID,
    confirmation: 0,
    from_external: true,
  };
}

/**
 * PX4's gimbal manager only obeys DO_GIMBAL_MANAGER_PITCHYAW from its primary control
 * (source_system/component, else DENIED), and CONFIGURE is not source-checked. This makes the GCS
 * (255/190, the source of every VehicleCommand) the primary control. Sent before EVERY Gimbal: a
 * mission gimbal action or the RC can take the control, and the next move would be DENIED silently.
 * @param {object} args
 * @param {object} [args.position] current position (attributes.systemId)
 * @returns {object} px4_msgs/msg/VehicleCommand
 */
export function buildGimbalTakeControl({ position }) {
  const system = systemIdOf(position, 'Gimbal');
  return message(VEHICLE_CMD.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE, { system, component: MAV_COMP_ID_ALL }, {
    param1: GCS_SYSTEM_ID, // primary control sysid
    param2: GCS_COMPONENT_ID, // primary control compid
    param3: LEAVE_UNCHANGED, // secondary control sysid
    param4: LEAVE_UNCHANGED, // secondary control compid
    param7: ALL_GIMBAL_DEVICES,
  });
}

/**
 * Body-frame relative move → absolute target.
 * dx forward, dy right, dz up (m), dyaw clockwise (deg); heading = position.course (deg, 0 = north).
 * @returns {{latitude:number, longitude:number, altitude:number, yaw:number}} yaw in rad
 */
export function relativeTarget(position, { dx = 0, dy = 0, dz = 0, dyaw = 0 }) {
  const h = (position.course ?? 0) * DEG_TO_RAD;
  const east = dx * Math.sin(h) + dy * Math.cos(h);
  const north = dx * Math.cos(h) - dy * Math.sin(h);
  const { latitude, longitude } = offsetLatLon(position.latitude, position.longitude, east, north);
  return {
    latitude,
    longitude,
    altitude: position.altitude + dz,
    yaw: wrapPi(((position.course ?? 0) + dyaw) * DEG_TO_RAD),
  };
}

/**
 * @param {object} args
 * @param {string} args.type        a CommandType commandRegistry implements with a VehicleCommand
 * @param {object} [args.attributes] command attributes, ALREADY validated and defaulted by
 *   command_schema (commandsModel.sendCommand): numbers in range, defaults filled in
 * @param {object} [args.position]  current position ({latitude, longitude, altitude AMSL, course,
 *   attributes.systemId})
 * @returns {object} px4_msgs/msg/VehicleCommand
 */
export function buildVehicleCommand({ type, attributes = {}, position }) {
  const vehicleMessage = (command, params, component = MAV_COMP_ID_AUTOPILOT1) =>
    message(command, { system: systemIdOf(position, type), component }, params);
  switch (type) {
    case CommandType.TAKEOFF: {
      requirePosition(position, type);
      const height = attributes.altitude; // m above the current altitude
      return vehicleMessage(VEHICLE_CMD.VEHICLE_CMD_NAV_TAKEOFF, {
        param4: wrapPi((position.course ?? 0) * DEG_TO_RAD),
        param5: position.latitude,
        param6: position.longitude,
        param7: position.altitude + height,
      });
    }
    case CommandType.LAND:
      return vehicleMessage(VEHICLE_CMD.VEHICLE_CMD_NAV_LAND);
    case CommandType.RETURN_HOME:
      return vehicleMessage(VEHICLE_CMD.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
    case CommandType.HOLD:
      return vehicleMessage(VEHICLE_CMD.VEHICLE_CMD_DO_SET_MODE, {
        param1: MODE_CUSTOM,
        param2: PX4_MAIN_MODE_AUTO,
        param3: PX4_SUB_MODE_AUTO_LOITER,
      });
    case CommandType.ARM:
      return vehicleMessage(VEHICLE_CMD.VEHICLE_CMD_COMPONENT_ARM_DISARM, { param1: 1 });
    case CommandType.DISARM:
      return vehicleMessage(VEHICLE_CMD.VEHICLE_CMD_COMPONENT_ARM_DISARM, { param1: 0 });
    case CommandType.KILL_MOTORS:
      return vehicleMessage(VEHICLE_CMD.VEHICLE_CMD_COMPONENT_ARM_DISARM, { param1: 0, param2: FORCE_DISARM_MAGIC });
    case CommandType.MOVE_RELATIVE: {
      requirePosition(position, type);
      const target = relativeTarget(position, attributes);
      return vehicleMessage(VEHICLE_CMD.VEHICLE_CMD_DO_REPOSITION, {
        param1: -1, // default ground speed
        param2: REPOSITION_CHANGE_MODE,
        param4: target.yaw,
        param5: target.latitude,
        param6: target.longitude,
        param7: target.altitude,
      });
    }
    case CommandType.GIMBAL: {
      // Degrees. Both axes always arrive: commandRegistry fills a missing one with the gimbal
      // telemetry, like DJI.
      const { pitch, yaw } = attributes;
      return vehicleMessage(
        VEHICLE_CMD.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        // Rates NaN = "angle only": with 0 PX4 ACKs ACCEPTED but the gimbal doesn't move (tested on the drone).
        { param1: pitch, param2: yaw, param3: NaN, param4: NaN, param5: GIMBAL_MANAGER_FLAGS_YAW_LOCK },
        MAV_COMP_ID_ALL
      );
    }
    default:
      throw new Error(`'${type}' is not a vehicle command`);
  }
}
