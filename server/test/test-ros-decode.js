import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { decodeRosMsg } from '../models/ros/rosDecode.js';
import { getDroneStatus, getArmingStatus, getVehicleCommand, getVehicleCmdResult } from '../config/status.js';

// What decodeRosMsg does for each (slot, messageType), one case per decoder. Born as
// characterization tests to prove the if-chain -> registry refactor changed nothing; the
// quirks they exposed (sonar type typo, per-category battery scale) have since been fixed.
//
// `type` is the subscriber slot of devices_msg.yaml. What differs per CATEGORY arrives as
// `params` (the category's own subscriber entry): `scale`, `unit`, `heading_offset`.
// The decoder never sees a category name.

const dec = (o) => decodeRosMsg({ deviceId: 7, ...o });
const near = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) < eps, `${a} is not within ${eps} of ${b}`);

// 90° yaw rotation about z, and identity.
const IDENTITY = { w: 1, x: 0, y: 0, z: 0 };
const YAW_90 = { w: Math.SQRT1_2, x: 0, y: 0, z: Math.SQRT1_2 };

describe('decodeRosMsg: position family', () => {
  test('position / NavSatFix', () => {
    const r = dec({
      type: 'position',
      msgType: 'sensor_msgs/NavSatFix',
      msg: { header: { seq: 5 }, latitude: 37.1, longitude: -5.9, altitude: 12.34 },
    });
    assert.equal(r.id, 5);
    assert.equal(r.deviceId, 7);
    assert.equal(r.latitude, 37.1);
    assert.equal(r.longitude, -5.9);
    assert.equal(r.altitude, 12.3);
    assert.equal(typeof r.deviceTime, 'string');
  });

  test('position / Odometry gives a local position only', () => {
    const r = dec({
      type: 'position',
      msgType: 'nav_msgs/msg/Odometry',
      msg: { pose: { pose: { position: { x: 1, y: 2, z: 3 } } } },
    });
    assert.deepEqual(r, { deviceId: 7, localposition: { x: 1, y: 2, z: 3 } });
  });

  test('position_global / px4 VehicleGlobalPosition', () => {
    const r = dec({ type: 'position_global', msgType: 'px4_msgs/msg/VehicleGlobalPosition', msg: { lat: 1, lon: 2, alt: 99.96 } });
    assert.equal(r.id, 0);
    assert.equal(r.latitude, 1);
    assert.equal(r.longitude, 2);
    assert.equal(r.altitude, 100);
    assert.equal(typeof r.deviceTime, 'string');
  });

  test('local_position / psdk PositionFused', () => {
    const r = dec({ type: 'local_position', msgType: 'psdk_interfaces/msg/PositionFused', msg: { position: { x: 1, y: 2, z: 3 } } });
    assert.deepEqual(r, { id: 0, deviceId: 7, localposition: { x: 1, y: 2, z: 3 } });
  });

  test('local_position / PointStamped', () => {
    const r = dec({ type: 'local_position', msgType: 'geometry_msgs/PointStamped', msg: { point: { x: 4, y: 5, z: 6 } } });
    assert.deepEqual(r, { id: 0, deviceId: 7, localposition: { x: 4, y: 5, z: 6 } });
  });

  test('local_position / px4 VehicleLocalPosition adds speed and course', () => {
    const r = dec({
      type: 'local_position',
      msgType: 'px4_msgs/msg/VehicleLocalPosition',
      msg: { x: 1, y: 2, z: 3, vx: 3, vy: 4, vz: 0, heading: 1 },
    });
    assert.deepEqual(r.localposition, { x: 1, y: 2, z: 3 });
    assert.equal(r.speed, 5);
    assert.equal(r.course, 57.2958);
    assert.equal(typeof r.deviceTime, 'string');
  });
});

describe('decodeRosMsg: px4 status', () => {
  test('vehicle_status', () => {
    const r = dec({
      type: 'vehicle_status',
      msgType: 'px4_msgs/msg/VehicleStatus',
      msg: { arming_state: 2, nav_state: 4, failsafe: false, pre_flight_checks_pass: true },
    });
    assert.deepEqual(r, {
      deviceId: 7,
      armState: getArmingStatus(2),
      navState: getDroneStatus(4),
      failsafe: false,
      flightCheck: true,
    });
  });

  test('vehicle_command_ack', () => {
    const r = dec({ type: 'vehicle_command_ack', msgType: 'px4_msgs/msg/VehicleCommandAck', msg: { command: 400, result: 0 } });
    assert.deepEqual(r, { deviceId: 7, commandAck: getVehicleCommand(400), resultCmdAck: getVehicleCmdResult(0) });
  });
});

describe('decodeRosMsg: heading (imu / heading / attitude)', () => {
  test('imu generic: identity quaternion is course 90', () => {
    const r = dec({ type: 'imu', msgType: 'sensor_msgs/Imu', msg: { orientation: IDENTITY } });
    assert.deepEqual(r, { deviceId: 7, course: 90 });
  });

  test('imu generic: 90° yaw is course 0', () => {
    const r = dec({ type: 'imu', msgType: 'sensor_msgs/Imu', msg: { orientation: YAW_90 } });
    assert.equal(r.course, 0);
  });

  test('imu with params.heading_offset (sjtu: -90) shifts the course by that offset', () => {
    const params = { heading_offset: -90 };
    const r = dec({ type: 'imu', msgType: 'sensor_msgs/Imu', msg: { orientation: IDENTITY }, params });
    assert.deepEqual(r, { deviceId: 7, course: -90 });

    const yawed = dec({ type: 'imu', msgType: 'sensor_msgs/Imu', msg: { orientation: YAW_90 }, params });
    assert.equal(yawed.course, -180);
  });

  test('heading / Float64 and Float32 pass data through as course', () => {
    assert.deepEqual(dec({ type: 'heading', msgType: 'std_msgs/Float64', msg: { data: 123.4 } }), { deviceId: 7, course: 123.4 });
    assert.deepEqual(dec({ type: 'heading', msgType: 'std_msgs/Float32', msg: { data: 5 } }), { deviceId: 7, course: 5 });
  });

  test('attitude / QuaternionStamped: 0 is north, 90 is east, wrapped to [-180, 180]', () => {
    const att = (q) => dec({ type: 'attitude', msgType: 'geometry_msgs/msg/QuaternionStamped', msg: { quaternion: q } });
    assert.equal(att(IDENTITY).course, 90);
    const deg45 = { w: Math.cos(Math.PI / 8), x: 0, y: 0, z: Math.sin(Math.PI / 8) };
    assert.equal(att(deg45).course, 45);
    const minus135 = { w: Math.cos((-3 * Math.PI) / 8), x: 0, y: 0, z: Math.sin((-3 * Math.PI) / 8) };
    assert.equal(att(minus135).course, -135);
  });
});

describe('decodeRosMsg: speed', () => {
  test('speed / Vector3Stamped, ROS1 and ROS2 names, is the rounded norm', () => {
    for (const msgType of ['geometry_msgs/Vector3Stamped', 'geometry_msgs/msg/Vector3Stamped']) {
      assert.deepEqual(dec({ type: 'speed', msgType, msg: { vector: { x: 3, y: 4, z: 0 } } }), { deviceId: 7, speed: 5 });
      assert.equal(dec({ type: 'speed', msgType, msg: { vector: { x: 1, y: 1, z: 1 } } }).speed, 1.7);
    }
  });

  test('speed / TwistStamped uses twist.linear', () => {
    const r = dec({ type: 'speed', msgType: 'geometry_msgs/TwistStamped', msg: { twist: { linear: { x: 0, y: 3, z: 4 } } } });
    assert.deepEqual(r, { deviceId: 7, speed: 5 });
  });

  test('mission_state / WaypointV2MissionStatePush reports velocity (cm/s) as speed', () => {
    const r = dec({ type: 'mission_state', msgType: 'dji_osdk_ros/WaypointV2MissionStatePush', msg: { velocity: 250 } });
    assert.deepEqual(r, { deviceId: 7, speed: 2.5 });
  });
});

describe('decodeRosMsg: battery (the category only contributes params.scale)', () => {
  const bat = (msgType, msg, params) => dec({ type: 'battery', msgType, msg, params });

  test('percentage is multiplied by params.scale', () => {
    const r = bat('aerialcore_common/BatteryState', { percentage: 0.876 }, { scale: 100 });
    near(r.batteryLevel, 87.6);
    assert.equal(r.deviceId, 7);
    assert.deepEqual(bat('sensor_msgs/msg/BatteryState', { percentage: 0.5 }, { scale: 100 }), { deviceId: 7, batteryLevel: 50 });
  });

  test('without params.scale the percentage is left untouched', () => {
    assert.deepEqual(bat('sensor_msgs/BatteryState', { percentage: 0.87 }), { deviceId: 7, batteryLevel: 0.87 });
    assert.deepEqual(bat('multiuav_interfaces/BatteryState', { percentage: 64 }, {}), { deviceId: 7, batteryLevel: 64 });
  });

  test('px4_msgs BatteryStatus: remaining * 100, whatever the params', () => {
    assert.deepEqual(bat('px4_msgs/msg/BatteryStatus', { remaining: 0.5 }), { deviceId: 7, batteryLevel: 50 });
    assert.deepEqual(bat('px4_msgs/msg/BatteryStatus', { remaining: 0.5 }, { scale: 100 }), { deviceId: 7, batteryLevel: 50 });
  });
});

describe('decodeRosMsg: gimbal and obstacles', () => {
  test('gimbal / Vector3Stamped (ROS1) passes the vector through', () => {
    const vector = { x: 1, y: 2, z: 3 };
    const r = dec({ type: 'gimbal', msgType: 'geometry_msgs/Vector3Stamped', msg: { vector } });
    assert.deepEqual(r, { deviceId: 7, gimbal: vector });
  });

  test('gimbal with params.unit "rad" (psdk) converts radians to degrees', () => {
    const r = dec({
      type: 'gimbal',
      msgType: 'geometry_msgs/msg/Vector3Stamped',
      msg: { vector: { x: Math.PI, y: 0, z: -Math.PI / 2 } },
      params: { unit: 'rad' },
    });
    near(r.gimbal.x, 180);
    near(r.gimbal.y, 0);
    near(r.gimbal.z, -90);
  });

  test('gimbal / ROS2 Vector3Stamped without a unit is passed through', () => {
    const vector = { x: 1, y: 2, z: 3 };
    const r = dec({ type: 'gimbal', msgType: 'geometry_msgs/msg/Vector3Stamped', msg: { vector } });
    assert.deepEqual(r, { deviceId: 7, gimbal: vector });
  });

  test('obstacle_info / dji and psdk messages are passed through raw', () => {
    for (const msgType of ['dji_osdk_ros/ObstacleInfo', 'psdk_interfaces/msg/RelativeObstacleInfo']) {
      const msg = { down: 1, front: 2 };
      assert.equal(dec({ type: 'obstacle_info', msgType, msg }).obstacle_info, msg);
    }
  });

  test('obstacle_info / sensor_msgs/Range is a downward reading, and the old typo no longer matches', () => {
    assert.deepEqual(dec({ type: 'obstacle_info', msgType: 'sensor_msgs/Range', msg: { range: 1.5 } }), {
      deviceId: 7,
      obstacle_info: { down: 1.5 },
    });
    assert.equal(dec({ type: 'obstacle_info', msgType: 'sensor_msg/Range', msg: { range: 1.5 } }), null);
  });
});

describe('decodeRosMsg: camera, flight status, state machine', () => {
  test('camera / CompressedImage, ROS1 and ROS2 names, decodes base64 into a Buffer', () => {
    for (const msgType of ['sensor_msgs/CompressedImage', 'sensor_msgs/msg/CompressedImage']) {
      const r = dec({ type: 'camera', msgType, msg: { data: 'AQID' } });
      assert.ok(Buffer.isBuffer(r.camera));
      assert.deepEqual([...r.camera], [1, 2, 3]);
      assert.equal(r.deviceId, 7);
    }
  });

  test('flight_status / UInt8 hardcodes protocol dji', () => {
    assert.deepEqual(dec({ type: 'flight_status', msgType: 'std_msgs/UInt8', msg: { data: 2 } }), {
      deviceId: 7,
      protocol: 'dji',
      landed_state: 2,
    });
  });

  test('state_machine / std_msgs/String hardcodes protocol catec', () => {
    assert.deepEqual(dec({ type: 'state_machine', msgType: 'std_msgs/String', msg: { data: 'HOVER' } }), {
      deviceId: 7,
      protocol: 'catec',
      mission_state: '0',
      wp_reached: '0',
      uav_state: 'ok',
      landed_state: 'HOVER',
    });
  });

  test('state_machine / UAVState takes the protocol from airframe_type', () => {
    const r = dec({
      type: 'state_machine',
      msgType: 'muav_state_machine/UAVState',
      msg: { airframe_type: 'px4', mission_state: '1', wp_reached: '2', uav_state: 'ok', landed_state: 3 },
    });
    assert.deepEqual(r, { deviceId: 7, protocol: 'px4', mission_state: '1', wp_reached: '2', uav_state: 'ok', landed_state: 3 });
  });
});

describe('decodeRosMsg: slots that ignore the message type', () => {
  test('humidity and threat pass data through whatever the msgType is', () => {
    assert.deepEqual(dec({ type: 'humidity', msgType: 'std_msgs/Int32MultiArray', msg: { data: [1, 2] } }), {
      deviceId: 7,
      sensors_humidity: [1, 2],
    });
    assert.deepEqual(dec({ type: 'threat', msgType: 'std_msgs/Bool', msg: { data: true } }), { deviceId: 7, threat: true });
    assert.deepEqual(dec({ type: 'threat', msgType: undefined, msg: { data: 2 } }), { deviceId: 7, threat: 2 });
  });
});

describe('decodeRosMsg: unknown combinations', () => {
  test('returns null for declared-but-undecoded slots and for wrong message types', () => {
    assert.equal(dec({ type: 'height_above_takeoff', msgType: 'std_msgs/Float32', msg: { data: 1 } }), null);
    assert.equal(dec({ type: 'vo_position', msgType: 'dji_osdk_ros/VOPosition', msg: {} }), null);
    assert.equal(dec({ type: 'flight_status', msgType: 'sensor_msgs/JointState', msg: {} }), null);
    assert.equal(dec({ type: 'position', msgType: 'px4_msgs/msg/VehicleGlobalPosition', msg: {} }), null);
    assert.equal(dec({ type: 'nope', msgType: 'nope', msg: {} }), null);
  });
});
