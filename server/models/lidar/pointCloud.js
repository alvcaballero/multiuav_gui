// Pure helpers for a sensor_msgs/PointCloud2 on its way to a browser: read it by its own `fields`,
// shrink it to a budget of points, pack it for the WebSocket. No ROS, no devices, no sockets.

// sensor_msgs/PointField datatype → DataView getter and size
const DATATYPE = {
  1: ['getInt8', 1],
  2: ['getUint8', 1],
  3: ['getInt16', 2],
  4: ['getUint16', 2],
  5: ['getInt32', 4],
  6: ['getUint32', 4],
  7: ['getFloat32', 4],
  8: ['getFloat64', 8],
};

/**
 * What the viewer can ask for: a ceiling of points and the shortest gap between two frames. The
 * lidar publishes ~5.7k points at ~10 Hz (≈1.8 MB/s before rosbridge's base64), which a browser
 * cannot take for a fleet; each level cuts both the points and the rate.
 */
export const LIDAR_LEVELS = Object.freeze({
  low: { id: 0, maxPoints: 1000, intervalMs: 500 },
  medium: { id: 1, maxPoints: 4000, intervalMs: 333 },
  high: { id: 2, maxPoints: 12000, intervalMs: 200 },
});

export const DEFAULT_LIDAR_LEVEL = 'medium';

// rosbridge sends a uint8[] as a base64 string; a bridge that does not would send the plain array.
function toBytes(data) {
  if (typeof data === 'string') return Buffer.from(data, 'base64');
  if (Array.isArray(data) || ArrayBuffer.isView(data)) return Buffer.from(data);
  throw new Error('PointCloud2.data is neither base64 nor bytes');
}

/**
 * Reads x/y/z (and intensity when the cloud has it) wherever the message says they are, so it does not
 * depend on the sensor's layout. Points with a non-finite coordinate (no return) are dropped.
 * @returns {{ count: number, xyz: Float32Array, intensity: Float32Array }}
 */
export function parsePointCloud2(msg) {
  const { width, height, point_step: step, row_step: rowStep, fields, is_bigendian: bigEndian } = msg ?? {};
  if (!(width > 0) || !(height > 0) || !(step > 0) || !Array.isArray(fields)) {
    throw new Error('not a PointCloud2');
  }
  const field = (name) => fields.find((f) => f.name === name);
  const [fx, fy, fz, fi] = ['x', 'y', 'z', 'intensity'].map(field);
  if (!fx || !fy || !fz) throw new Error('PointCloud2 without x/y/z');
  for (const f of [fx, fy, fz, fi]) {
    if (f && !DATATYPE[f.datatype]) throw new Error(`PointCloud2 field ${f.name} has an unknown datatype ${f.datatype}`);
  }

  const bytes = toBytes(msg.data);
  const rows = height;
  const stride = rowStep > 0 ? rowStep : width * step;
  if (bytes.length < (rows - 1) * stride + width * step) throw new Error('PointCloud2.data is shorter than its size');

  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const little = !bigEndian;
  const read = (f, base) => view[DATATYPE[f.datatype][0]](base + f.offset, little);

  const xyz = new Float32Array(width * height * 3);
  const intensity = new Float32Array(width * height);
  let n = 0;
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < width; c++) {
      const base = r * stride + c * step;
      const x = read(fx, base);
      const y = read(fy, base);
      const z = read(fz, base);
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
      xyz[n * 3] = x;
      xyz[n * 3 + 1] = y;
      xyz[n * 3 + 2] = z;
      intensity[n] = fi ? read(fi, base) : 0;
      n++;
    }
  }
  return { count: n, xyz: xyz.subarray(0, n * 3), intensity: intensity.subarray(0, n) };
}

// 17 bits per axis keep the key under 2^53; two voxels 131072 cells apart would share a key, which
// at a lidar's range only costs a point.
const AXIS = 0x1ffff;
const voxelKey = (ix, iy, iz) => (((ix & AXIS) * 131072 + (iy & AXIS)) * 131072) + (iz & AXIS);

function pick(cloud, indices) {
  const xyz = new Float32Array(indices.length * 3);
  const intensity = new Float32Array(indices.length);
  indices.forEach((src, dst) => {
    xyz[dst * 3] = cloud.xyz[src * 3];
    xyz[dst * 3 + 1] = cloud.xyz[src * 3 + 1];
    xyz[dst * 3 + 2] = cloud.xyz[src * 3 + 2];
    intensity[dst] = cloud.intensity[src];
  });
  return { count: indices.length, xyz, intensity };
}

function voxelIndices(cloud, size) {
  const seen = new Set();
  const kept = [];
  for (let i = 0; i < cloud.count; i++) {
    const key = voxelKey(
      Math.floor(cloud.xyz[i * 3] / size),
      Math.floor(cloud.xyz[i * 3 + 1] / size),
      Math.floor(cloud.xyz[i * 3 + 2] / size)
    );
    if (seen.has(key)) continue;
    seen.add(key);
    kept.push(i);
  }
  return kept;
}

// The search stops when the cell keeps at least this share of the budget: closer is not worth another pass.
const FILL_TARGET = 0.9;
const SEARCH_ROUNDS = 16;

/**
 * At most `maxPoints` points, and close to that many, spread over the space (a voxel grid keeps one
 * point per cell, so a dense wall near the drone does not eat the budget of what is farther). The size
 * of the cell is searched by bisection for the smallest one that fits: the count is then smooth from
 * one frame to the next, instead of jumping between "all of them" and "a few" around the ceiling
 * (a growing cell that overshoots left 720 points of a 4000-point budget, and the view flickered).
 * If no cell fits, the last resort is an even stride, so the ceiling always holds.
 */
export function reducePoints(cloud, maxPoints) {
  if (cloud.count <= maxPoints) return cloud;

  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < cloud.count; i++) {
    for (let a = 0; a < 3; a++) {
      const v = cloud.xyz[i * 3 + a];
      if (v < min[a]) min[a] = v;
      if (v > max[a]) max[a] = v;
    }
  }
  // A cell as big as the cloud keeps one point; one far smaller than any spacing keeps them all. Sizes
  // span orders of magnitude, so the middle is geometric.
  let tooSmall = 1e-3;
  let big = Math.max(max[0] - min[0], max[1] - min[1], max[2] - min[2], 2 * tooSmall);
  let best = null;
  for (let round = 0; round < SEARCH_ROUNDS; round++) {
    const kept = voxelIndices(cloud, Math.sqrt(tooSmall * big));
    if (kept.length <= maxPoints) {
      best = kept;
      big = Math.sqrt(tooSmall * big);
      if (kept.length >= maxPoints * FILL_TARGET) break;
    } else {
      tooSmall = Math.sqrt(tooSmall * big);
    }
  }
  if (best) return pick(cloud, best);

  const stride = cloud.count / maxPoints;
  return pick(cloud, Array.from({ length: maxPoints }, (_, i) => Math.floor(i * stride)));
}
