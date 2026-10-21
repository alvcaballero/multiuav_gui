import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { EventEmitter } from 'node:events';
import { MqttServer } from '../models/mqtt/MqttServer.js';
import { encodeMsg } from '../models/msgEncode.js';

// Real JSON published by the drone-side mqtt_client fork (px4_ros2 mock, 2026-10-14), kept
// verbatim (excerpted) so a change in the wire shape or in a decoder shows up here.
// Caveat from the drone side: the mock reports zeros and `*_valid: false`; these samples prove
// the SHAPE, not flight values. battery / vehicle_command_ack have no real sample yet.

const SAMPLES = {
  position_global: {
    timestamp: 1791991691302621,
    lat: 37.193646,
    lon: -6.70293,
    alt: 0.0,
    lat_lon_valid: false,
    alt_valid: false,
    eph: 0.0,
    epv: 0.0,
  },
  local_position: {
    timestamp: 1791991691302621,
    xy_valid: true,
    x: 1.5,
    y: -2.5,
    z: -3.0,
    delta_xy: [0.0, 0.0],
    vx: 3.0,
    vy: 4.0,
    vz: 0.0,
    heading: 0.0,
    ref_lat: 37.193646,
    ref_lon: -6.70293,
  },
  vehicle_status: { timestamp: 1791991691302621, arming_state: 1, nav_state: 0, failsafe: false, system_id: 1, pre_flight_checks_pass: true },
};

const CATEGORY = {
  subscribers: {
    position_global: { messageType: 'px4_msgs/msg/VehicleGlobalPosition' },
    local_position: { messageType: 'px4_msgs/msg/VehicleLocalPosition' },
    vehicle_status: { messageType: 'px4_msgs/msg/VehicleStatus' },
  },
};

function setup() {
  const client = Object.assign(new EventEmitter(), { connected: true, publish() {}, subscribe() {}, end() {} });
  const updates = [];
  new MqttServer('mqtt://test', {
    connect: () => client,
    deps: {
      devices: { getDevice: async () => null, getByName: async () => ({ id: 7, name: 'zz_test', category: 'px4_ros2' }) },
      positions: { updatePosition: (p) => updates.push(p) },
      categories: { getCategory: () => CATEGORY },
    },
  });
  const droneSays = async (slot, body) => {
    client.emit('message', `muav/zz_test/pub/${slot}`, Buffer.from(JSON.stringify(body)));
    await new Promise((r) => setImmediate(r));
  };
  return { updates, droneSays };
}

describe('px4_ros2 telemetry as the fork publishes it', () => {
  test('position_global: lat/lon/alt', async () => {
    const { updates, droneSays } = setup();
    await droneSays('position_global', SAMPLES.position_global);
    assert.equal(updates[0].deviceId, 7);
    assert.equal(updates[0].latitude, 37.193646);
    assert.equal(updates[0].longitude, -6.70293);
    assert.equal(updates[0].altitude, 0);
  });

  test('local_position: local xyz, course from heading, ground speed from vx/vy/vz', async () => {
    const { updates, droneSays } = setup();
    await droneSays('local_position', SAMPLES.local_position);
    assert.deepEqual(updates[0].localposition, { x: 1.5, y: -2.5, z: -3 });
    assert.equal(updates[0].speed, 5);
    assert.equal(updates[0].course, 0);
  });

  test('vehicle_status: arming and nav state, and the system id VehicleCommand targets', async () => {
    const { updates, droneSays } = setup();
    await droneSays('vehicle_status', SAMPLES.vehicle_status);
    assert.equal(updates[0].armState, 'DISARMED');
    assert.equal(updates[0].flightCheck, true);
    assert.equal(updates[0].failsafe, false);
    assert.equal(updates[0].systemId, 1);
  });
});

describe('configure_mission request vs muav_gcs_interfaces/srv/LoadMission', () => {
  // Fields of Mission.msg (the srv's request is a single field called `request`).
  const MISSION_FIELDS = [
    'type', 'waypoint', 'radius', 'vel_max', 'vel_idle', 'yaw', 'gimbal_pitch', 'speed',
    'yaw_mode', 'trace_mode', 'gimbal_pitch_mode', 'finish_action', 'command_list', 'command_parameter',
  ];
  const task = {
    task_id: 'T1',
    device: 'zz_test',
    action: 'ROUTE',
    params: { max_vel: 5, idle_vel: 3, mode_yaw: 0, mode_gimbal: 0, mode_trace: 0, mode_landing: 1 },
    wp: [
      { type: 'takeoff', pos: [37.19, -6.7, 10], yaw: 0 },
      { type: 'inspection', pos: [37.2, -6.71, 20], yaw: 90, gimbal: -30, speed: 4 },
    ],
  };

  test('the body is { request: <Mission> } with exactly the Mission.msg fields', () => {
    const body = encodeMsg('muav_gcs_interfaces/srv/LoadMission', task);
    assert.deepEqual(Object.keys(body), ['request']);
    assert.deepEqual(Object.keys(body.request).sort(), [...MISSION_FIELDS].sort());
    assert.equal(body.request.waypoint.length, 2);
  });
});
