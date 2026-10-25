import { test } from 'node:test';
import assert from 'node:assert/strict';
import { encodeLidarFrame, MSG_TYPE_LIDAR_FRAME, POINT_FLOATS } from '../models/lidar/lidarFrame.js';
import { LIDAR_LEVELS as SERVER_LEVELS, DEFAULT_LIDAR_LEVEL as SERVER_DEFAULT } from '../models/lidar/pointCloud.js';
import {
  decodeLidarFrame,
  MSG_TYPE_LIDAR_FRAME as CLIENT_MSG_TYPE,
  POINT_FLOATS as CLIENT_POINT_FLOATS,
  LIDAR_LEVELS as CLIENT_LEVELS,
  DEFAULT_LIDAR_LEVEL as CLIENT_DEFAULT,
} from '../../client/src/common/util/lidarFrame.js';

// The client has no test runner: the util it decodes with is tested from here, against the server's encoder.

const cloud = {
  count: 3,
  xyz: new Float32Array([1, 2, 3, -4, 5, 6, 7, 8, 9]),
  intensity: new Float32Array([10, 20, 30]),
};
const toArrayBuffer = (buf) => buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);

test('the client mirrors the wire constants and the levels of the server', () => {
  assert.equal(CLIENT_MSG_TYPE, MSG_TYPE_LIDAR_FRAME);
  assert.equal(CLIENT_POINT_FLOATS, POINT_FLOATS);
  assert.equal(CLIENT_DEFAULT, SERVER_DEFAULT);
  assert.deepEqual(
    CLIENT_LEVELS.map(({ key, id }) => [key, id]),
    Object.entries(SERVER_LEVELS).map(([key, level]) => [key, level.id])
  );
});

test('the client reads what the server writes, whatever the length of the device id', () => {
  for (const deviceId of ['1', '12', '123', '1234', 'uav_12345']) {
    for (const level of Object.keys(SERVER_LEVELS)) {
      const frame = decodeLidarFrame(toArrayBuffer(encodeLidarFrame({ deviceId, level, cloud })));
      assert.equal(frame.deviceId, deviceId);
      assert.equal(frame.level, level);
      assert.equal(frame.count, 3);
      assert.deepEqual([...frame.points], [1, 2, 3, 10, -4, 5, 6, 20, 7, 8, 9, 30]);
    }
  }
});

test('the client ignores what is not a lidar frame or is cut short', () => {
  const frame = toArrayBuffer(encodeLidarFrame({ deviceId: '2', level: 'low', cloud }));
  assert.equal(decodeLidarFrame(new Uint8Array([1, 1, 50, 255]).buffer), null, 'a camera frame (type 1)');
  assert.equal(decodeLidarFrame(frame.slice(0, frame.byteLength - 1)), null, 'cut short');
  assert.equal(decodeLidarFrame('text'), null);
});
