// PX4 VehicleCommand builder — command type (config/commandTypes.js) → px4_msgs/msg/VehicleCommand.
//
// Pure: receives the device's current position and returns the full wire message, so it
// is testable without ROS. commandsModel publishes the result on the device's
// `VehicleCommand` publisher (/fmu/in/vehicle_command).
//
// Notes that are easy to get wrong:
//   - The message goes straight to uORB, so angles are RADIANS (navigator writes
//     param4 into position_setpoint.yaw as-is), unlike the MAVLink spec's degrees.
//   - rosbridge carries JSON: NaN ("keep current") can't be sent. Every param that
//     PX4 would read as NaN is filled with the current value instead.
//   - target_system is the vehicle's MAV_SYS_ID, different per drone (multi-vehicle SITL:
//     instance + 1). It comes from telemetry (vehicle_status.system_id →
//     position.attributes.systemId). Unknown → the command is REJECTED, never sent as 0
//     (broadcast): a KillMotors would reach every vehicle on the link.
//     target_component 1 = MAV_COMP_ID_AUTOPILOT1.
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

function message(command, targetSystem, params = {}) {
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
    target_system: targetSystem,
    target_component: 1,
    source_system: 255,
    source_component: 190,
    confirmation: 0,
    from_external: true,
  };
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
  const vehicleMessage = (command, params) => message(command, systemIdOf(position, type), params);
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
      const { pitch, yaw } = attributes;
      return vehicleMessage(VEHICLE_CMD.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW, {
        param1: pitch, //pitch
        param2: yaw, // yaw
        param5: 0, // 0 =  relative to device,  64 =  relative to world
      });
    }
    default:
      throw new Error(`'${type}' is not a vehicle command`);
  }
}
