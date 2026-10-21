import { getDatetime, round } from '../common/utils.js';
import { getVehicleCommand } from '../config/status.js';
import { px4ArmState, px4FlightMode, px4CommandResult, djiFlightStatus, djiFlightMode, catecFlightState, landedStateOf } from './vehicleStates.js';

// ROS message -> internal telemetry patch (`TelemetryPatch`, models/positions/positionReducers.js).
//
// A decoder is picked by (slot, messageType): the slot is the key under `subscribers` in
// devices_msg.yaml and the messageType is what that entry declares. What differs between
// CATEGORIES (a scale, a unit, an offset) is NOT code: it is a param on the category's own
// subscriber entry, and arrives here as `params`. Never branch on the category name.
//
//   battery.scale        multiplier that turns `percentage` into percent (default 1)
//   gimbal.unit          'rad' converts the angles to degrees (default: already degrees)
//   imu.heading_offset   degrees added to the yaw to get the course (default 90)

const toDeg = (rad) => (rad * 180) / Math.PI;
const norm3 = ({ x, y, z }) => Math.sqrt(x * x + y * y + z * z);

const ANY = '*';
const decoders = new Map(); // `${slot}|${messageType}` -> ({ msg, deviceId, params }) => patch | null
const keyOf = (slot, msgType) => `${slot}|${msgType}`;

function register(slot, msgTypes, decode) {
  for (const msgType of [msgTypes].flat()) {
    const key = keyOf(slot, msgType);
    if (decoders.has(key)) throw new Error(`msgDecode: duplicate decoder for ${key}`);
    decoders.set(key, decode);
  }
}

// Declared in devices_msg.yaml on purpose, but nothing consumes it: say so instead of
// leaving a hole that looks like a typo.
const ignore = (slot, msgTypes) => register(slot, msgTypes, () => null);

/**
 * slot → the concrete msgTypes it decodes (an explicit `ignore` included). A wildcard slot
 * (registered with ANY) accepts any type, so it contributes no concrete type here.
 * @returns {Map<string, string[]>}
 */
export function decodableMsgTypes() {
  const out = new Map();
  for (const key of decoders.keys()) {
    const [slot, msgType] = key.split('|');
    if (msgType === ANY) continue;
    (out.get(slot) ?? out.set(slot, []).get(slot)).push(msgType);
  }
  return out;
}

/** True when (slot, msgType) has a decoder, an explicit `ignore` included. */
export function hasDecoder(slot, msgType) {
  return decoders.has(keyOf(slot, msgType)) || decoders.has(keyOf(slot, ANY));
}

/**
 * @param {object} ctx
 * @param {object} ctx.msg - ROS message as rosbridge delivers it
 * @param {number|string} ctx.deviceId
 * @param {string} ctx.type - subscriber slot (`position`, `battery`, ...)
 * @param {string} ctx.msgType - messageType declared by the category for that slot
 * @param {object} [ctx.params] - the category's subscriber entry (scale, unit, heading_offset, ...)
 * @returns {object|null} a TelemetryPatch, or null when there is nothing to apply
 */
export function decodeMsg({ msg, deviceId, type, msgType, params = {} }) {
  const decode = decoders.get(keyOf(type, msgType)) ?? decoders.get(keyOf(type, ANY));
  return decode ? decode({ msg, deviceId, params }) : null;
}

// ─── position ──────────────────────────────────────────────────────────────────

register('position', 'sensor_msgs/NavSatFix', ({ msg, deviceId }) => ({
  id: msg.header.seq,
  deviceId,
  latitude: msg.latitude,
  longitude: msg.longitude,
  altitude: round(msg.altitude, 1),
  deviceTime: getDatetime(), // "2023-03-09T22:12:44.000+00:00",
}));

register('position', 'nav_msgs/msg/Odometry', ({ msg, deviceId }) => ({
  deviceId,
  localposition: { x: msg.pose.pose.position.x, y: msg.pose.pose.position.y, z: msg.pose.pose.position.z },
}));

register('position_global', 'px4_msgs/msg/VehicleGlobalPosition', ({ msg, deviceId }) => ({
  id: 0,
  deviceId,
  latitude: msg.lat,
  longitude: msg.lon,
  altitude: round(msg.alt, 1),
  deviceTime: getDatetime(),
}));

register('local_position', 'psdk_interfaces/msg/PositionFused', ({ msg, deviceId }) => ({
  id: 0,
  deviceId,
  localposition: { x: msg.position.x, y: msg.position.y, z: msg.position.z },
}));

register('local_position', 'geometry_msgs/PointStamped', ({ msg, deviceId }) => ({
  id: 0,
  deviceId,
  localposition: { x: msg.point.x, y: msg.point.y, z: msg.point.z },
}));

register('local_position', 'px4_msgs/msg/VehicleLocalPosition', ({ msg, deviceId }) => ({
  id: 0,
  deviceId,
  localposition: { x: msg.x, y: msg.y, z: msg.z },
  course: msg.heading * 57.2958, // radians to degrees
  speed: round(norm3({ x: msg.vx, y: msg.vy, z: msg.vz }), 1),
  deviceTime: getDatetime(),
}));

// ─── px4 status ────────────────────────────────────────────────────────────────

register('vehicle_status', 'px4_msgs/msg/VehicleStatus', ({ msg, deviceId }) => ({
  deviceId,
  armState: px4ArmState(msg.arming_state),
  flightMode: px4FlightMode(msg.nav_state),
  flightModeRaw: msg.nav_state,
  failsafe: msg.failsafe,
  flightCheck: msg.pre_flight_checks_pass,
  systemId: msg.system_id, // MAV_SYS_ID: the target_system of every VehicleCommand to it
}));

register('vehicle_command_ack', 'px4_msgs/msg/VehicleCommandAck', ({ msg, deviceId }) => ({
  deviceId,
  commandAck: getVehicleCommand(msg.command),
  commandAckRaw: msg.command,
  resultCmdAck: px4CommandResult(msg.result),
  resultCmdAckRaw: msg.result,
}));

// ─── heading ───────────────────────────────────────────────────────────────────

register('imu', 'sensor_msgs/Imu', ({ msg, deviceId, params }) => {
  // https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
  const q = msg.orientation;
  const yaw = Math.atan2(2.0 * (q.z * q.w + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * -1;
  return { deviceId, course: Math.round((params.heading_offset ?? 90) + yaw * 57.295) };
});

register('heading', ['std_msgs/Float64', 'std_msgs/Float32'], ({ msg, deviceId }) => ({ deviceId, course: msg.data }));

register('attitude', 'geometry_msgs/msg/QuaternionStamped', ({ msg, deviceId }) => {
  const q = msg.quaternion;
  const yawRad = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  // 0 is north and 90 is east, normalised to [-180, 180] like ROS does.
  let yaw = 90 - toDeg(yawRad);
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
  return { deviceId, course: round(yaw, 1) };
});

// ─── speed ─────────────────────────────────────────────────────────────────────

register('speed', ['geometry_msgs/Vector3Stamped', 'geometry_msgs/msg/Vector3Stamped'], ({ msg, deviceId }) => ({
  deviceId,
  speed: round(norm3(msg.vector), 1),
}));

register('speed', 'geometry_msgs/TwistStamped', ({ msg, deviceId }) => ({
  deviceId,
  speed: round(norm3(msg.twist.linear), 1),
}));

register('mission_state', 'dji_osdk_ros/WaypointV2MissionStatePush', ({ msg, deviceId }) => ({
  deviceId,
  speed: round(msg.velocity * 0.01, 1),
}));

// ─── battery ───────────────────────────────────────────────────────────────────

register('battery', 'px4_msgs/msg/BatteryStatus', ({ msg, deviceId }) => ({
  deviceId,
  batteryLevel: msg.remaining * 100,
}));

// Every other battery message is sensor_msgs/BatteryState-shaped (`percentage`), whatever
// its package is called; whether that is 0-1 or 0-100 is the category's `scale`.
register('battery', ANY, ({ msg, deviceId, params }) => ({
  deviceId,
  batteryLevel: msg.percentage * (params.scale ?? 1),
}));

// ─── gimbal and obstacles ──────────────────────────────────────────────────────

register(
  'gimbal',
  ['geometry_msgs/Vector3Stamped', 'geometry_msgs/msg/Vector3Stamped'],
  ({ msg, deviceId, params }) => {
    const v = msg.vector;
    return { deviceId, gimbal: params.unit === 'rad' ? { x: toDeg(v.x), y: toDeg(v.y), z: toDeg(v.z) } : v };
  }
);

// PX4: q = [w, x, y, z] in FRD/NED, so its ZYX Euler angles are pitch up-positive and yaw
// clockwise from north, as the other gimbals report them. Roll and pitch are always earth frame.
// Yaw is earth frame only with YAW_LOCK / YAW_IN_EARTH_FRAME; otherwise it is relative to the
// vehicle and delta_yaw (rad, the vehicle heading) makes it absolute. With neither the absolute
// yaw is unknown and left out, so the cached one stays.
const GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16;
const GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME = 64;
const wrap180 = (deg) => ((((deg + 180) % 360) + 360) % 360) - 180;

register('gimbal', 'px4_msgs/msg/GimbalDeviceAttitudeStatus', ({ msg, deviceId }) => {
  const [w, x, y, z] = msg.q;
  const roll = toDeg(Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)));
  const pitch = toDeg(Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))));
  const yaw = toDeg(Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)));
  const earthYaw = msg.device_flags & (GIMBAL_DEVICE_FLAGS_YAW_LOCK | GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME);
  let absoluteYaw;
  if (earthYaw) absoluteYaw = wrap180(yaw);
  else if (Number.isFinite(msg.delta_yaw)) absoluteYaw = wrap180(yaw + toDeg(msg.delta_yaw));
  return { deviceId, gimbal: { x: pitch, y: roll, z: absoluteYaw } };
});

register(
  'obstacle_info',
  ['dji_osdk_ros/ObstacleInfo', 'psdk_interfaces/msg/RelativeObstacleInfo'],
  ({ msg, deviceId }) => ({ deviceId, obstacle_info: msg })
);

register('obstacle_info', 'sensor_msgs/Range', ({ msg, deviceId }) => ({
  deviceId,
  obstacle_info: { down: msg.range },
}));

register('obstacle_info', 'muav_gcs_interfaces/msg/ObstacleInfo', ({ msg, deviceId }) => {
  const obstacle_info = {
    front: msg.front.distance,
    back: msg.back.distance,
    left: msg.left.distance,
    right: msg.right.distance,
    up: msg.up.distance,
    down: msg.down.distance,
  };
  return { deviceId, obstacle_info };
});
// ─── camera, flight status, state machine ──────────────────────────────────────

register('camera', ['sensor_msgs/CompressedImage', 'sensor_msgs/msg/CompressedImage'], ({ msg, deviceId }) => ({
  // rosbridge serializes uint8[] fields as base64 — decode once here so the rest
  // of the pipeline (cache, HTTP snapshot, WS broadcast) deals in real bytes.
  deviceId,
  camera: Buffer.from(msg.data, 'base64'),
}));

register('flight_status', 'std_msgs/UInt8', ({ msg, deviceId }) => ({
  deviceId,
  ...djiFlightStatus(msg.data),
  flightPhaseRaw: msg.data,
}));

// Same message type as flight_status, a different thing: the flight mode, not the phase.
register('display_mode', 'std_msgs/UInt8', ({ msg, deviceId }) => ({
  deviceId,
  flightMode: djiFlightMode(msg.data),
  flightModeRaw: msg.data,
}));

register('state_machine', 'std_msgs/String',({ msg, deviceId }) => ({
  deviceId,
  mission_state: '0',
  wp_reached: '0',
  uav_state: 'ok',
  ...catecFlightState(msg.data),
  flightPhaseRaw: msg.data,
}));

register('state_machine', 'muav_state_machine/UAVState', ({ msg, deviceId }) => ({
  deviceId,
  mission_state: msg.mission_state,
  wp_reached: msg.wp_reached,
  uav_state: msg.uav_state,
  // A message without landed_state says nothing about the phase: it must not wipe the cached one.
  ...(msg.landed_state == null ? {} : { ...landedStateOf(msg.airframe_type, msg.landed_state), flightPhaseRaw: msg.landed_state }),
}));

// ─── slots whose payload does not depend on the message type ───────────────────

register('humidity', ANY, ({ msg, deviceId }) => ({ deviceId, sensors_humidity: msg.data }));
register('threat', ANY, ({ msg, deviceId }) => ({ deviceId, threat: msg.data }));

// ─── declared in devices_msg.yaml, nothing consumes them ───────────────────────

ignore('height_above_takeoff', ['std_msgs/Float32', 'mavros_msgs/Altitude']);
ignore('vo_position', 'dji_osdk_ros/VOPosition');
ignore('flight_status', 'sensor_msgs/JointState');
