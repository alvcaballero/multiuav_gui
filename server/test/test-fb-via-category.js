import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import * as fb from 'fbmsglib';
import * as flatbuffers from 'flatbuffers';
import { categoryModel } from '../models/category.js';
import { decodeMsg } from '../models/msgDecode.js';
import { decodeFbMsg, decodeFbViaCategory } from '../models/flatbuffer/fbDecode.js';
import { fbMsgAsRos, rosFieldName, rosShape } from '../models/flatbuffer/fbUnpack.js';

// ROS is the reference: a robofleet message must decode to exactly what the same message delivered
// by rosbridge decodes to. These tests build real robofleet buffers (object API → pack, the layout
// the bridge sends) and compare against `decodeMsg` on the equivalent ROS JSON.

const F = fb.fb;
const subscribers = categoryModel.getCategory('simulationFB').subscribers;
const DEVICE = 7;

const header = () => {
  const h = new F.std_msgs.HeaderT();
  h.stamp = new F.RosTimeT();
  h.frameId = 'map';
  return h;
};
const vec3 = (x, y, z) => Object.assign(new F.geometry_msgs.Vector3T(), { x, y, z });

// Packs `table` and returns what FlatbufferServer._onMessage has in hand: the buffer and its metadata.
function robofleet(table, type, topic) {
  table._Metadata = Object.assign(new F.MsgMetadataT(), { type, topic });
  const b = new flatbuffers.Builder(256);
  b.finish(table.pack(b));
  const buf = new flatbuffers.ByteBuffer(b.asUint8Array());
  return { buf, metadata: F.MsgWithMetadata.getRootAsMsgWithMetadata(buf)._Metadata() };
}

const viaCategory = ({ buf, metadata }) =>
  decodeFbViaCategory({ type: metadata.type(), topic: metadata.topic(), buf, deviceId: DEVICE, subscribers });
const legacy = ({ buf, metadata }) => decodeFbMsg(metadata, buf, DEVICE, 'simulationFB', 'uav1');
const asRos = (slot, msg) =>
  decodeMsg({ msg, deviceId: DEVICE, type: slot, msgType: subscribers[slot].messageType, params: subscribers[slot] });
const noTime = ({ deviceTime, ...rest }) => rest;

const cases = {
  position() {
    const t = new F.sensor_msgs.NavSatFixT();
    Object.assign(t, { header: header(), status: new F.sensor_msgs.NavSatStatusT(), positionCovariance: new Array(9).fill(0) });
    Object.assign(t, { latitude: 37.5, longitude: -6.7, altitude: 42.26 });
    return {
      slot: 'position',
      msg: robofleet(t, 'sensor_msgs/NavSatFix', '/uav1/gps_position'),
      ros: { header: { seq: 0 }, latitude: 37.5, longitude: -6.7, altitude: 42.26 },
    };
  },
  heading() {
    const t = Object.assign(new F.std_msgs.Float64T(), { data: 123.5 });
    return { slot: 'heading', msg: robofleet(t, 'std_msgs/Float64', '/uav1/orientation'), ros: { data: 123.5 } };
  },
  imu() {
    const t = new F.sensor_msgs.ImuT();
    Object.assign(t, {
      header: header(),
      orientation: Object.assign(new F.geometry_msgs.QuaternionT(), { x: 0, y: 0, z: 0.5, w: 0.8660254037844386 }),
      orientationCovariance: [],
      angularVelocity: vec3(0, 0, 0),
      angularVelocityCovariance: [],
      linearAcceleration: vec3(0, 0, 0),
      linearAccelerationCovariance: [],
    });
    return {
      slot: 'imu',
      msg: robofleet(t, 'sensor_msgs/Imu', '/uav1/imu'),
      ros: { orientation: { x: 0, y: 0, z: 0.5, w: 0.8660254037844386 } },
    };
  },
  battery() {
    const t = Object.assign(new F.sensor_msgs.BatteryStateT(), { header: header(), percentage: 87, cellVoltage: [], cellTemperature: [] });
    return { slot: 'battery', msg: robofleet(t, 'sensor_msgs/BatteryState', '/uav1/battery_state'), ros: { percentage: 87 } };
  },
  gimbal() {
    const t = Object.assign(new F.geometry_msgs.Vector3StampedT(), { header: header(), vector: vec3(-30, 0, 45) });
    return {
      slot: 'gimbal',
      msg: robofleet(t, 'geometry_msgs/Vector3Stamped', '/uav1/gimbal'),
      ros: { vector: { x: -30, y: 0, z: 45 } },
    };
  },
  speed() {
    const t = Object.assign(new F.geometry_msgs.Vector3StampedT(), { header: header(), vector: vec3(3, 4, 12) });
    return {
      slot: 'speed',
      msg: robofleet(t, 'geometry_msgs/Vector3Stamped', '/uav1/speed'),
      ros: { vector: { x: 3, y: 4, z: 12 } },
    };
  },
  flight_status() {
    const t = Object.assign(new F.std_msgs.UInt8T(), { data: 1 });
    return { slot: 'flight_status', msg: robofleet(t, 'std_msgs/UInt8', '/uav1/flight_status'), ros: { data: 1 } };
  },
  display_mode() {
    const t = Object.assign(new F.std_msgs.UInt8T(), { data: 6 });
    return { slot: 'display_mode', msg: robofleet(t, 'std_msgs/UInt8', '/uav1/display_mode'), ros: { data: 6 } };
  },
  threat() {
    const t = Object.assign(new F.std_msgs.UInt8T(), { data: 2 });
    return { slot: 'threat', msg: robofleet(t, 'std_msgs/UInt8', '/uav1/alarm'), ros: { data: 2 } };
  },
  obstacle_info() {
    const t = Object.assign(new F.dji_osdk_ros.ObstacleInfoT(), { down: 1.5, front: 2.5, right: 3.5, back: 4.5, left: 5.5, up: 6.5 });
    // decodeMsg hands the whole message on, so the reference is the full ROS message
    const ros = {
      header: null,
      ...{ down: 1.5, front: 2.5, right: 3.5, back: 4.5, left: 5.5, up: 6.5 },
      ...{ down_health: 0, front_health: 0, right_health: 0, back_health: 0, left_health: 0, up_health: 0 },
    };
    return { slot: 'obstacle_info', msg: robofleet(t, 'dji_osdk_ros/ObstacleInfo', '/uav1/obstacle_info'), ros };
  },
  mission_state() {
    const t = Object.assign(new F.dji_osdk_ros.WaypointV2MissionStatePushT(), { velocity: 500 });
    return {
      slot: 'mission_state',
      msg: robofleet(t, 'dji_osdk_ros/WaypointV2MissionStatePush', '/uav1/mission_state'),
      ros: { velocity: 500 },
    };
  },
};

describe('robofleet through the category == the same ROS message through msgDecode', () => {
  for (const [name, build] of Object.entries(cases)) {
    test(name, () => {
      const { slot, msg, ros } = build();
      const got = viaCategory(msg);
      assert.ok(got, `${name} did not go through the category`);
      assert.deepEqual(noTime(got), noTime(asRos(slot, ros)));
    });
  }

  test('position keeps what the legacy decoder gave (and adds what ROS adds)', () => {
    const { msg } = cases.position();
    const got = viaCategory(msg);
    assert.equal(got.latitude, legacy(msg).latitude);
    assert.equal(got.longitude, legacy(msg).longitude);
    assert.equal(got.altitude, 42.3, 'rounded to 1 decimal like ROS');
    assert.ok(got.deviceTime, 'the ROS decoder stamps deviceTime');
  });

  test('heading and the enum slots match the legacy decoder', () => {
    for (const name of ['heading', 'flight_status', 'display_mode', 'threat']) {
      const { msg } = cases[name]();
      const old = legacy(msg);
      const got = viaCategory(msg);
      for (const key of Object.keys(old).filter((k) => k !== 'deviceId')) {
        assert.deepEqual(got[key], old[key], `${name}.${key}`);
      }
    }
  });

  test('obstacle_info carries the six distances the legacy decoder gave, plus the rest of the message', () => {
    const { msg } = cases.obstacle_info();
    const old = legacy(msg).obstacle_info;
    const got = viaCategory(msg).obstacle_info;
    for (const side of Object.keys(old)) assert.equal(got[side], old[side], side);
    assert.equal(Object.keys(got).some((k) => k.startsWith('healt')), false, 'schema constants are not fields');
  });
});

// Where the legacy decoder disagreed with ROS, ROS wins. Pinned so the change is deliberate.
describe('deliberate differences from the legacy decoder (ROS is the reference)', () => {
  test('speed is 3D like ROS (legacy: horizontal only, as a string)', () => {
    const { msg } = cases.speed();
    assert.equal(legacy(msg).speed, '5.00');
    assert.equal(viaCategory(msg).speed, 13);
  });

  test('mission_state velocity is scaled by 0.01 like ROS (legacy: raw, as speed)', () => {
    const { msg } = cases.mission_state();
    assert.equal(legacy(msg).speed, 500);
    assert.equal(viaCategory(msg).speed, 5);
  });

  test('imu course uses the ROS formula and heading_offset', () => {
    const { msg } = cases.imu();
    assert.notEqual(viaCategory(msg).course, legacy(msg).course);
  });

  test('battery is a number, not a string', () => {
    const { msg } = cases.battery();
    assert.equal(legacy(msg).batteryLevel, '87');
    assert.equal(viaCategory(msg).batteryLevel, 87);
  });
});

describe('what does NOT go through the category (falls back to decodeFbMsg)', () => {
  test('a topic the category does not declare', () => {
    const t = Object.assign(new F.std_msgs.Float64T(), { data: 1 });
    assert.equal(viaCategory(robofleet(t, 'std_msgs/Float64', '/uav1/something_else')), undefined);
  });

  test('a declared topic that carries another type', () => {
    const t = Object.assign(new F.std_msgs.Float64T(), { data: 1 });
    assert.equal(viaCategory(robofleet(t, 'std_msgs/Float64', '/uav1/gps_position')), undefined);
  });

  test('a device whose category has no subscribers', () => {
    const { msg } = cases.heading();
    const { metadata, buf } = msg;
    const out = decodeFbViaCategory({ type: metadata.type(), topic: metadata.topic(), buf, deviceId: 1, subscribers: undefined });
    assert.equal(out, undefined);
  });

  test('a ROS2 style type in the category still matches the robofleet type', () => {
    const t = Object.assign(new F.std_msgs.Float64T(), { data: 9 });
    const { metadata, buf } = robofleet(t, 'std_msgs/Float64', '/uav1/orientation');
    const subs = { heading: { name: '/orientation', messageType: 'std_msgs/msg/Float64' } };
    // no decoder for (heading, std_msgs/msg/Float64): must not blow up, must fall back
    assert.equal(decodeFbViaCategory({ type: metadata.type(), topic: metadata.topic(), buf, deviceId: 1, subscribers: subs }), undefined);
  });
});

describe('field names: back to what the .msg calls them', () => {
  test('snake_case fields come back, camelCase ones in the .fbs stay as they are', () => {
    assert.equal(rosFieldName('frameId'), 'frame_id');
    assert.equal(rosFieldName('positionCovariance'), 'position_covariance');
    assert.equal(rosFieldName('curWaypointIndex'), 'curWaypointIndex');
    assert.equal(rosFieldName('maxVel'), 'maxVel');
    assert.equal(rosFieldName('latitude'), 'latitude');
  });

  test('rosShape renames recursively and drops the robofleet metadata', () => {
    const out = rosShape({ _Metadata: { type: 'x' }, header: { _Metadata: null, frameId: 'map', seq: 1 }, list: [{ frameId: 'a' }] });
    assert.deepEqual(out, { header: { frame_id: 'map', seq: 1 }, list: [{ frame_id: 'a' }] });
  });

  test('every field of every table in fbmsglib has a .msg name (none falls back to a guess)', () => {
    const tables = [];
    const visit = (ns, path) => {
      for (const [name, cls] of Object.entries(ns)) {
        if (typeof cls === 'function' && name.endsWith('T')) tables.push([`${path}${name}`, cls]);
        else if (cls && typeof cls === 'object') visit(cls, `${path}${name}.`);
      }
    };
    visit(F, 'fb.');
    assert.ok(tables.length > 50, `found ${tables.length} tables`);

    const unknown = [];
    for (const [path, Cls] of tables) {
      for (const key of Object.keys(new Cls()).filter((k) => k !== '_Metadata')) {
        if (rosFieldName(key) === null) unknown.push(`${path}.${key}`);
      }
    }
    assert.deepEqual(unknown, []);
  });

  test('the unpacked message has no robofleet metadata and uses .msg names', () => {
    const { msg } = cases.position();
    const ros = fbMsgAsRos(msg.metadata.type(), msg.buf);
    assert.equal('_Metadata' in ros, false);
    assert.equal(ros.header.frame_id, 'map');
    assert.equal(ros.position_covariance.length, 9);
  });
});
