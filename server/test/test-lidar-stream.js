import { test, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { LidarStream, LIDAR_REJECT } from '../models/lidar/lidarStream.js';
import { LIDAR_LEVELS } from '../models/lidar/pointCloud.js';

const FIELDS = [
  { name: 'x', offset: 0, datatype: 7, count: 1 },
  { name: 'y', offset: 4, datatype: 7, count: 1 },
  { name: 'z', offset: 8, datatype: 7, count: 1 },
];
const cloud = (n) => {
  const buf = Buffer.alloc(n * 16);
  for (let i = 0; i < n; i++) {
    buf.writeFloatLE(i % 100, i * 16);
    buf.writeFloatLE(Math.floor(i / 100), i * 16 + 4);
    buf.writeFloatLE(1, i * 16 + 8);
  }
  return { width: n, height: 1, point_step: 16, row_step: n * 16, is_bigendian: false, fields: FIELDS, data: buf.toString('base64') };
};

const DEVICE = { id: 2, name: 'uav_1', category: 'px4_ros2' };

let stream, clock, opened, closed, sent, statuses, onMessage, connected, openResult;

beforeEach(() => {
  stream = new LidarStream();
  clock = 1_000_000;
  opened = [];
  closed = [];
  sent = [];
  statuses = [];
  connected = new Set(['a', 'b']);
  openResult = { state: 'success' };
  onMessage = null;
  stream.start({
    getDevice: async (id) => (String(id) === '2' ? DEVICE : null),
    supports: (device) => device.id === 2,
    open: (device, opts) => {
      opened.push({ device, throttleMs: opts.throttleMs });
      onMessage = opts.onMessage;
      return openResult;
    },
    close: (device) => closed.push(device),
    isConnected: (id) => connected.has(id),
    send: (clientId, buffer) => (sent.push({ clientId, buffer }), true),
    emitStatus: (status) => statuses.push(status),
    now: () => clock,
  });
});

test('nothing is open until a viewer asks, and the topic is throttled to its level', async () => {
  assert.equal(opened.length, 0);
  const result = await stream.subscribe('a', 2, 'low');
  assert.deepEqual(result, { ok: true, level: 'low' });
  assert.equal(opened.length, 1);
  assert.equal(opened[0].throttleMs, LIDAR_LEVELS.low.intervalMs);
  assert.deepEqual(statuses.at(-1), { clientId: 'a', deviceId: 2, ok: true, level: 'low' });
});

test('one ROS subscription serves every client; it closes with the last viewer', async () => {
  await stream.subscribe('a', 2, 'low');
  await stream.subscribe('b', 2, 'high');
  assert.equal(opened.length, 2, 'reopened once, for the quicker level');
  assert.equal(opened[1].throttleMs, LIDAR_LEVELS.high.intervalMs);

  stream.unsubscribe('b', 2);
  assert.equal(closed.length, 0, 'a viewer is still there');
  assert.equal(opened.at(-1).throttleMs, LIDAR_LEVELS.low.intervalMs, 'and the topic is relaxed to its level');

  stream.removeClient('a');
  assert.equal(closed.length, 1);
  assert.equal(stream.viewers().size, 0);
});

test('each client gets the frame of its own level, and a level is not sent faster than its interval', async () => {
  await stream.subscribe('a', 2, 'low');
  await stream.subscribe('b', 2, 'high');

  onMessage(cloud(5000));
  assert.deepEqual(sent.map((s) => s.clientId).sort(), ['a', 'b']);
  const sizeOf = (id) => sent.find((s) => s.clientId === id).buffer.length;
  assert.ok(sizeOf('a') < sizeOf('b'), 'low is lighter than high');

  sent.length = 0;
  clock += LIDAR_LEVELS.high.intervalMs; // high is due again, low (500 ms) is not
  onMessage(cloud(5000));
  assert.deepEqual(sent.map((s) => s.clientId), ['b']);

  sent.length = 0;
  clock += LIDAR_LEVELS.low.intervalMs;
  onMessage(cloud(5000));
  assert.deepEqual(sent.map((s) => s.clientId).sort(), ['a', 'b']);
});

test('a message that is not due is not even parsed', async () => {
  await stream.subscribe('a', 2, 'low');
  onMessage(cloud(100));
  sent.length = 0;
  clock += 10;
  onMessage('this would throw if it were parsed');
  assert.equal(sent.length, 0);
});

test('refuses with the reason, and opens nothing', async () => {
  assert.equal((await stream.subscribe('a', 99, 'low')).reason, LIDAR_REJECT.UNKNOWN_DEVICE);
  assert.equal((await stream.subscribe('a', 2, 'ultra')).reason, LIDAR_REJECT.INVALID);
  assert.equal((await stream.subscribe('a', null)).reason, LIDAR_REJECT.INVALID);
  stream._deps.supports = () => false;
  assert.equal((await stream.subscribe('a', 2, 'low')).reason, LIDAR_REJECT.NOT_SUPPORTED);
  assert.equal(opened.length, 0);
  assert.equal(stream.viewers().size, 0);
});

test('when ROS cannot open the topic the viewer is not kept', async () => {
  openResult = { state: 'error', msg: 'ROS no conectado' };
  const result = await stream.subscribe('a', 2, 'low');
  assert.equal(result.reason, LIDAR_REJECT.ROS_UNAVAILABLE);
  assert.equal(stream.viewers().size, 0);
});

test('a client that left while the device was looked up is not registered', async () => {
  const pending = stream.subscribe('a', 2, 'low');
  connected.delete('a');
  await pending;
  assert.equal(stream.viewers().size, 0);
  assert.equal(opened.length, 0);
});

test('a device that changed ends its viewers, and ROS coming back reopens the rest', async () => {
  await stream.subscribe('a', 2, 'low');
  stream.reopenAll();
  assert.equal(opened.length, 2, 'ROS came back: opened again for the same viewer');

  stream.deviceGone(2);
  assert.equal(statuses.at(-1).reason, LIDAR_REJECT.DEVICE_CHANGED);
  assert.equal(closed.length, 1);
  assert.equal(stream.viewers().size, 0);
});

test('a broken cloud is reported once and the stream keeps going', async () => {
  await stream.subscribe('a', 2, 'low');
  onMessage({ nonsense: true });
  clock += 1000;
  onMessage({ nonsense: true });
  assert.equal(sent.length, 0);
  clock += 1000;
  onMessage(cloud(100));
  assert.equal(sent.length, 1);
});

test('the raw inspector opens the topic and gets a summary, never the cloud', async () => {
  const published = [];
  stream._deps.publishRaw = (message) => published.push(message);

  await stream.inspect('a', 2, 'lidar_3d_points');
  assert.equal(opened.length, 1, 'inspecting opens the topic');
  assert.equal(opened[0].throttleMs, 1000);

  onMessage({ ...cloud(5000), header: { frame_id: 'lidar' } });
  assert.equal(published.length, 1);
  const { deviceId, slot, raw } = published[0];
  assert.deepEqual([deviceId, slot], [2, 'lidar_3d_points']);
  assert.equal(raw.data, undefined, 'the buffer stays out');
  assert.equal(raw.points, 5000);
  assert.equal(raw.data_bytes, 5000 * 16);
  assert.equal(raw.header.frame_id, 'lidar');
  assert.equal(raw.sample.length, 8);
  assert.deepEqual(raw.sample[1], { x: 1, y: 0, z: 1, intensity: 0 });
  assert.equal(sent.length, 0, 'and no binary frame is sent to anybody');

  clock += 100; // inside its second
  onMessage(cloud(10));
  assert.equal(published.length, 1);

  stream.uninspect('a', 2, 'lidar_3d_points');
  assert.equal(closed.length, 1, 'the topic closes with the last reader');
});

test('the inspector ignores other slots and shares the topic with a viewer', async () => {
  await stream.inspect('a', 2, 'battery');
  assert.equal(opened.length, 0);

  await stream.subscribe('b', 2, 'low');
  await stream.inspect('a', 2, 'lidar_3d_points');
  stream.unsubscribe('b', 2);
  assert.equal(closed.length, 0, 'the inspector is still reading');
  stream.removeClient('a');
  assert.equal(closed.length, 1);
});
