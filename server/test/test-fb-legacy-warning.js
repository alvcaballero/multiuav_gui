import { test, mock } from 'node:test';
import assert from 'node:assert/strict';
import * as fb from 'fbmsglib';
import * as flatbuffers from 'flatbuffers';
import { FlatbufferServer } from '../models/flatbuffer/FlatbufferServer.js';
import { devicesController } from '../controllers/devices.js';
import { positionsController } from '../controllers/positions.js';
import { logger } from '../common/logger.js';

// decodeFbMsg is transitional: the server says, once per device and topic, that a message still
// needs it, so the log shows when nothing does and it can be deleted (TODO.md).

const device = (id, category = 'dji_M210_noetic') => ({ id, name: `uav${id}`, category });
const metadata = (topic, type = 'sensor_msgs/NavSatFix') => ({ topic: () => topic, type: () => type });

test('the legacy decoder is reported once per device and topic, not once per message', () => {
  const server = new FlatbufferServer(0);
  const warn = mock.method(logger, 'warn', () => {});
  try {
    for (let i = 0; i < 50; i++) server._warnLegacyDecoder(device(1), metadata('/uav1/gps_position'));
    assert.equal(warn.mock.callCount(), 1, '50 messages, one warning');

    server._warnLegacyDecoder(device(1), metadata('/uav1/orientation', 'std_msgs/Float64'));
    assert.equal(warn.mock.callCount(), 2, 'another topic of the same device');

    server._warnLegacyDecoder(device(2), metadata('/uav2/gps_position'));
    assert.equal(warn.mock.callCount(), 3, 'the same topic name on another device');
  } finally {
    warn.mock.restore();
    server.disconnect();
  }
});

test('the warning says what to do about it', () => {
  const server = new FlatbufferServer(0);
  const warn = mock.method(logger, 'warn', () => {});
  try {
    server._warnLegacyDecoder(device(3, 'simulationFB'), metadata('/uav3/dji_speed', 'geometry_msgs/Vector3Stamped'));
    const [message] = warn.mock.calls[0].arguments;
    assert.match(message, /uav3/);
    assert.match(message, /simulationFB/);
    assert.match(message, /\/uav3\/dji_speed/);
    assert.match(message, /geometry_msgs\/Vector3Stamped/);
    assert.match(message, /Declare the topic/);
  } finally {
    warn.mock.restore();
    server.disconnect();
  }
});

// ── The wiring in _onMessage: the warning only appears when the legacy decoder is what runs ──

const F = fb.fb;

// A real robofleet NavSatFix on `topic`, as a device would send it.
function navSatFix(topic) {
  const header = new F.std_msgs.HeaderT();
  header.stamp = new F.RosTimeT();
  header.frameId = 'map';
  const t = Object.assign(new F.sensor_msgs.NavSatFixT(), {
    header,
    status: new F.sensor_msgs.NavSatStatusT(),
    positionCovariance: new Array(9).fill(0),
    latitude: 37.5,
    longitude: -6.7,
    altitude: 42,
  });
  t._Metadata = Object.assign(new F.MsgMetadataT(), { type: 'sensor_msgs/NavSatFix', topic });
  const b = new flatbuffers.Builder(256);
  b.finish(t.pack(b));
  return Buffer.from(b.asUint8Array());
}

// Runs one message through the real _onMessage with the device lookup and the position cache stubbed.
async function receive(topic, category) {
  const server = new FlatbufferServer(0);
  const warn = mock.method(logger, 'warn', () => {});
  const lookup = mock.method(devicesController, 'getByName', async () => device(9, category));
  const update = mock.method(positionsController, 'updatePosition', () => {});
  try {
    await server._onMessage(navSatFix(topic), 'uav9');
    return {
      warnings: warn.mock.calls.map((c) => c.arguments[0]).filter((m) => /legacy per-type decoder/.test(m)),
      updates: update.mock.calls.map((c) => c.arguments[0]),
    };
  } finally {
    warn.mock.restore();
    lookup.mock.restore();
    update.mock.restore();
    server.disconnect();
  }
}

test('a topic the category declares is decoded by the category and does NOT warn', async () => {
  const { warnings, updates } = await receive('/uav9/gps_position', 'simulationFB');
  assert.deepEqual(warnings, []);
  assert.equal(updates.length, 1);
  assert.equal(updates[0].latitude, 37.5);
  assert.ok(updates[0].deviceTime, 'through the category it is stamped like a ROS position');
});

test('a topic the category does not declare still works through the legacy decoder, and warns once', async () => {
  const { warnings, updates } = await receive('/uav9/some_other_gps', 'simulationFB');
  assert.equal(warnings.length, 1);
  assert.match(warnings[0], /\/uav9\/some_other_gps/);
  assert.equal(updates.length, 1, 'telemetry is not lost while the bridge is not renamed');
  assert.equal(updates[0].latitude, 37.5);
  assert.equal(updates[0].deviceTime, undefined, 'the legacy decoder does not stamp it');
});

test('a device whose category declares ROS topics falls back and warns', async () => {
  const { warnings, updates } = await receive('/uav9/gps_position', 'dji_M210_noetic');
  assert.equal(warnings.length, 1);
  assert.match(warnings[0], /dji_M210_noetic/);
  assert.equal(updates.length, 1);
});
