// Mirror of the server's lidar frame (server/models/lidar/lidarFrame.js, pointCloud.js). The levels and
// the layout are checked against the server by server/test/test-lidar-frame-client.js.
//   [1 byte msgType = 2][1 byte len(deviceId)][1 byte level id][1 byte 0]
//   [deviceId UTF-8, zero-padded to a multiple of 4][uint32 LE count]
//   [count × (x, y, z, intensity) as float32 LE], in the sensor's frame (x forward, y left, z up), meters
export const MSG_TYPE_LIDAR_FRAME = 2;
export const POINT_FLOATS = 4;

/** What the viewer can ask for: the server cuts the points and the rate of each level. */
export const LIDAR_LEVELS = [
  { key: 'low', id: 0, label: 'Low' },
  { key: 'medium', id: 1, label: 'Medium' },
  { key: 'high', id: 2, label: 'High' },
];
export const DEFAULT_LIDAR_LEVEL = 'medium';

/**
 * Reads one binary WS message. `points` is a view on the message (no copy), interleaved x y z intensity.
 * @param {ArrayBuffer} buffer
 * @returns {{ deviceId: string, level: string, count: number, points: Float32Array } | null} null when it is
 *   not a lidar frame (the socket also carries camera frames) or it is cut short
 */
export function decodeLidarFrame(buffer) {
  if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < 8) return null;
  const bytes = new Uint8Array(buffer);
  if (bytes[0] !== MSG_TYPE_LIDAR_FRAME) return null;

  const idLength = bytes[1];
  const padded = Math.ceil(idLength / 4) * 4;
  const countAt = 4 + padded;
  if (buffer.byteLength < countAt + 4) return null;
  const count = new DataView(buffer).getUint32(countAt, true);
  const floatsAt = countAt + 4;
  if (buffer.byteLength < floatsAt + count * POINT_FLOATS * 4) return null;

  return {
    deviceId: new TextDecoder().decode(bytes.subarray(4, 4 + idLength)),
    level: LIDAR_LEVELS.find((level) => level.id === bytes[2])?.key ?? DEFAULT_LIDAR_LEVEL,
    count,
    points: new Float32Array(buffer, floatsAt, count * POINT_FLOATS),
  };
}
