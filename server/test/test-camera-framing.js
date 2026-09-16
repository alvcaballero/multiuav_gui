import { test } from 'node:test';
import assert from 'node:assert/strict';
import { encodeCameraFrame, MSG_TYPE_CAMERA_FRAME } from '../subscribers/cameraStreamSubscriber.js';

// Mirrors the client-side parser in SocketCameraCanvas.jsx — kept independent
// on purpose so this test breaks if the wire format drifts out of sync.
function decodeCameraFrame(buffer) {
  const bytes = new Uint8Array(buffer);
  const msgType = bytes[0];
  const idLen = bytes[1];
  const deviceId = Buffer.from(bytes.subarray(2, 2 + idLen)).toString('utf8');
  const jpeg = Buffer.from(bytes.subarray(2 + idLen));
  return { msgType, deviceId, jpeg };
}

test('encodeCameraFrame: round-trips deviceId and JPEG bytes', () => {
  const jpeg = Buffer.from([0xff, 0xd8, 0xff, 0xd9]); // minimal JPEG SOI/EOI markers
  const frame = encodeCameraFrame({ deviceId: 'uav_1', camera: jpeg });
  const decoded = decodeCameraFrame(frame);

  assert.equal(decoded.msgType, MSG_TYPE_CAMERA_FRAME);
  assert.equal(decoded.deviceId, 'uav_1');
  assert.deepEqual(decoded.jpeg, jpeg);
});

test('encodeCameraFrame: coerces a numeric deviceId to string', () => {
  const jpeg = Buffer.from([1, 2, 3]);
  const frame = encodeCameraFrame({ deviceId: 42, camera: jpeg });
  const decoded = decodeCameraFrame(frame);

  assert.equal(decoded.deviceId, '42');
});

test('encodeCameraFrame: header length matches the encoded deviceId byte length', () => {
  const deviceId = 'ünïcode_id'; // multi-byte UTF-8 chars — length in bytes != length in chars
  const frame = encodeCameraFrame({ deviceId, camera: Buffer.from([9]) });
  const idLen = frame[1];

  assert.equal(idLen, Buffer.byteLength(deviceId, 'utf8'));
  assert.notEqual(idLen, deviceId.length);
});

test('encodeCameraFrame: total length is header + deviceId + payload', () => {
  const deviceId = 'uav_7';
  const jpeg = Buffer.alloc(128, 0xaa);
  const frame = encodeCameraFrame({ deviceId, camera: jpeg });

  assert.equal(frame.length, 2 + Buffer.byteLength(deviceId, 'utf8') + jpeg.length);
});
