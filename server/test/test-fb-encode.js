import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import * as fb from 'fbmsglib';
import * as flatbuffers from 'flatbuffers';
import { encodeConfigMission, encodeSetBool, encodeFbMsg } from '../models/flatbuffer/fbEncode.js';
import { MissionToRos } from '../models/mission/missionEncodeConfig.js';
import { encodeMsg } from '../models/msgEncode.js';
import { devicesController } from '../controllers/devices.js';

// robofleet packs ROS messages: what reaches fbEncode is the ROS message of the msgType
// (encodeMsg, before the gateway), and it must come out of the Flatbuffer unchanged.
const TASK = {
  task_id: 'T1',
  device: 'uav1',
  uav_type: 'dji_M300',
  depends_on: [],
  params: { mode_landing: 2, mode_yaw: 2, mode_gimbal: 0, idle_vel: 7, max_vel: 12, mode_trace: 2 },
  wp: [
    { pos: [38.13898041, -3.17466429, 9], action: { photo: 0, gimbal: -90 }, yaw: 153.6 },
    { pos: [38.13816516, -3.17414854, 29.19], speed: 5 },
  ],
};
const META = { topic: '/uav1/configureMission', type: 'aerialcore_common/ConfigMission' };
const unpack = (bytes) =>
  fb.fb.aerialcore_common.ConfigMission.getRootAsConfigMission(new flatbuffers.ByteBuffer(bytes)).request();
const close = (actual, expected) =>
  assert.deepEqual(
    [...actual].map((v) => Math.round(v * 1000) / 1000),
    expected.map((v) => Math.round(v * 1000) / 1000)
  );

describe('encodeConfigMission — packs the ROS ConfigMission message', () => {
  const ros = MissionToRos(TASK);
  const req = unpack(encodeConfigMission(ros, META));

  test('waypoints and scalar fields', () => {
    assert.equal(req.waypointLength(), ros.waypoint.length);
    assert.ok(Math.abs(req.waypoint(0).latitude() - ros.waypoint[0].latitude) < 1e-9);
    assert.equal(req.maxVel(), ros.maxVel);
    assert.equal(req.idleVel(), ros.idleVel);
    assert.equal(req.yawMode(), ros.yawMode);
    assert.equal(req.finishAction(), ros.finishAction);
  });

  test('the { data } arrays of rosbridge JSON become the plain Flatbuffer vectors', () => {
    close(req.yawArray(), ros.yaw.data);
    close(req.speedArray(), ros.speed.data);
    close(req.gimbalPitchArray(), ros.gimbalPitch.data);
    assert.deepEqual([...req.commandListArray()], ros.commandList.data);
    close(req.commandParameterArray(), ros.commandParameter.data);
  });

  test('a v1 profile message without maxVel / gimbalPitchMode still packs', () => {
    // Same setup as test-mission-schema-contract: v1 (M210) with no params omits them.
    const v1 = MissionToRos({ ...TASK, uav_type: 'dji_M210_noetic', params: {} });
    assert.ok(!('maxVel' in v1));
    assert.equal(unpack(encodeConfigMission(v1, META)).maxVel(), 0);
  });
});

describe('encodeSetBool', () => {
  const unpackBool = (bytes) =>
    fb.fb.std_srvs.SetBool.getRootAsSetBool(new flatbuffers.ByteBuffer(bytes)).request().data();
  const meta = { topic: '/uav1/commandMission', type: 'std_srvs/SetBool' };

  test('packs the request data, and starts (true) when the request is empty', () => {
    assert.equal(unpackBool(encodeSetBool({ data: true }, meta)), true);
    assert.equal(unpackBool(encodeSetBool(encodeMsg('std_srvs/TriggerRequest', {}), meta)), true);
  });
});

describe('encodeFbMsg — mission type without a Flatbuffer schema', () => {
  test('a ROS2 LoadMission device is rejected instead of packed as ConfigMission', async () => {
    devicesController.getDevice = async () => ({ id: 1, name: 'px4_1', category: 'px4_ros2' });
    await assert.rejects(
      encodeFbMsg({ uav_id: 1, type: 'configure_mission', attributes: {} }),
      /no Flatbuffer schema for mission type 'muav_gcs_interfaces\/srv\/LoadMission'/
    );
  });
});
