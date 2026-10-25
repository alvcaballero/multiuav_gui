import { LIDAR_LEVELS } from './pointCloud.js';

// Wire format of a lidar frame, a binary WS message like the camera's (cameraStreamSubscriber.js,
// type 1) but with the numbers aligned, so the client can look at them without copying:
//   [1 byte msgType = 2][1 byte len(deviceId)][1 byte level id][1 byte 0]
//   [deviceId UTF-8, zero-padded to a multiple of 4][uint32 LE count]
//   [count × (x, y, z, intensity) as float32 LE]
// Coordinates are the sensor's own frame, in meters.
export const MSG_TYPE_LIDAR_FRAME = 2;

// The floats are written through a Float32Array, which uses the host's byte order.
if (new Uint8Array(new Uint32Array([1]).buffer)[0] !== 1) {
  throw new Error('lidarFrame needs a little-endian host');
}

export const POINT_FLOATS = 4;

/** @param {{ deviceId: string|number, level: keyof typeof LIDAR_LEVELS, cloud: { count: number, xyz: Float32Array, intensity: Float32Array } }} frame */
export function encodeLidarFrame({ deviceId, level, cloud }) {
  const idBytes = Buffer.from(String(deviceId), 'utf8');
  if (idBytes.length > 255) throw new Error('deviceId too long');
  const padded = Math.ceil(idBytes.length / 4) * 4;
  const header = Buffer.alloc(4 + padded + 4);
  header[0] = MSG_TYPE_LIDAR_FRAME;
  header[1] = idBytes.length;
  header[2] = LIDAR_LEVELS[level].id;
  idBytes.copy(header, 4);
  header.writeUInt32LE(cloud.count, 4 + padded);

  const points = new Float32Array(cloud.count * POINT_FLOATS);
  for (let i = 0; i < cloud.count; i++) {
    points[i * POINT_FLOATS] = cloud.xyz[i * 3];
    points[i * POINT_FLOATS + 1] = cloud.xyz[i * 3 + 1];
    points[i * POINT_FLOATS + 2] = cloud.xyz[i * 3 + 2];
    points[i * POINT_FLOATS + 3] = cloud.intensity[i];
  }
  return Buffer.concat([header, Buffer.from(points.buffer, points.byteOffset, points.byteLength)]);
}
