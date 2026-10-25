import { test } from 'node:test';
import assert from 'node:assert/strict';
import { parsePointCloud2, reducePoints, LIDAR_LEVELS } from '../models/lidar/pointCloud.js';
import { encodeLidarFrame, MSG_TYPE_LIDAR_FRAME } from '../models/lidar/lidarFrame.js';

// The layout of the simulated lidar: x y z float32, intensity at 16, ring uint16 at 24, step 32.
const FIELDS = [
  { name: 'x', offset: 0, datatype: 7, count: 1 },
  { name: 'y', offset: 4, datatype: 7, count: 1 },
  { name: 'z', offset: 8, datatype: 7, count: 1 },
  { name: 'intensity', offset: 16, datatype: 7, count: 1 },
  { name: 'ring', offset: 24, datatype: 4, count: 1 },
];

function cloudMsg(points, { width = points.length, height = 1, asBase64 = true } = {}) {
  const step = 32;
  const buf = Buffer.alloc(points.length * step);
  points.forEach(([x, y, z, i = 0], n) => {
    buf.writeFloatLE(x, n * step);
    buf.writeFloatLE(y, n * step + 4);
    buf.writeFloatLE(z, n * step + 8);
    buf.writeFloatLE(i, n * step + 16);
  });
  return {
    width,
    height,
    point_step: step,
    row_step: width * step,
    is_bigendian: false,
    fields: FIELDS,
    data: asBase64 ? buf.toString('base64') : [...buf],
  };
}

// A wall of points on a regular grid: surface-like, as a lidar sees it
const wall = (n) => Array.from({ length: n }, (_, i) => [(i % 100) * 0.05, 3, Math.floor(i / 100) * 0.05, i]);

test('reads x/y/z/intensity by the offsets of the message, from base64 or from plain bytes', () => {
  const points = [
    [1, 2, 3, 10],
    [-4, 5.5, 6, 20],
  ];
  for (const asBase64 of [true, false]) {
    const cloud = parsePointCloud2(cloudMsg(points, { asBase64 }));
    assert.equal(cloud.count, 2);
    assert.deepEqual([...cloud.xyz], [1, 2, 3, -4, 5.5, 6]);
    assert.deepEqual([...cloud.intensity], [10, 20]);
  }
});

test('drops the points with no return (NaN / Infinity)', () => {
  const cloud = parsePointCloud2(cloudMsg([[1, 1, 1], [NaN, 0, 0], [0, Infinity, 0], [2, 2, 2]]));
  assert.equal(cloud.count, 2);
  assert.deepEqual([...cloud.xyz], [1, 1, 1, 2, 2, 2]);
});

test('a cloud without intensity still parses (zeros)', () => {
  const msg = cloudMsg([[1, 2, 3]]);
  msg.fields = FIELDS.filter((f) => f.name !== 'intensity');
  assert.deepEqual([...parsePointCloud2(msg).intensity], [0]);
});

test('refuses what is not a cloud, instead of sending garbage', () => {
  assert.throws(() => parsePointCloud2(null), /not a PointCloud2/);
  assert.throws(() => parsePointCloud2({ ...cloudMsg([[1, 2, 3]]), fields: [FIELDS[0]] }), /x\/y\/z/);
  assert.throws(() => parsePointCloud2({ ...cloudMsg([[1, 2, 3]]), data: '' }), /shorter/);
  assert.throws(() => parsePointCloud2({ ...cloudMsg([[1, 2, 3]]), fields: [{ ...FIELDS[0], datatype: 99 }, FIELDS[1], FIELDS[2]] }), /datatype/);
});

test('reduce keeps the ceiling and spreads the points instead of cutting the cloud', () => {
  const cloud = parsePointCloud2(cloudMsg(wall(5760), { width: 360, height: 16 }));
  assert.equal(cloud.count, 5760);
  for (const [, level] of Object.entries(LIDAR_LEVELS)) {
    const out = reducePoints(cloud, level.maxPoints);
    assert.ok(out.count <= level.maxPoints, `${out.count} > ${level.maxPoints}`);
    if (level.maxPoints < cloud.count) {
      assert.ok(out.count > level.maxPoints * 0.3, `${out.count} is far below the budget ${level.maxPoints}`);
      // spread over the whole wall (x in [0, 4.95], z in [0, 2.85]), not just its first rows
      const xs = Array.from({ length: out.count }, (_, i) => out.xyz[i * 3]);
      const zs = Array.from({ length: out.count }, (_, i) => out.xyz[i * 3 + 2]);
      assert.ok(Math.max(...xs) > 4.5 && Math.max(...zs) > 2.5);
    }
  }
});

// What the simulator's lidar saw: ~4000 points, most of them a wall 5 m ahead, a few hundred far away
function simulatedScene(total) {
  let seed = 12345;
  const rand = () => ((seed = (seed * 1664525 + 1013904223) % 4294967296) / 4294967296);
  const far = Math.floor(total * 0.13);
  return Array.from({ length: total }, (_, i) =>
    i < total - far
      ? [4.7 + rand() * 0.4, -6 + rand() * 12, -10 + rand() * 12, 1] // the wall
      : [10 + rand() * 35, rand() * 40, -10.5 + rand() * 0.3, 1] // the ground, far
  );
}

test('the count does not jump around the ceiling (it flickered between ~700 and ~4000 points)', () => {
  const budget = LIDAR_LEVELS.medium.maxPoints;
  for (const total of [budget - 10, budget - 1, budget, budget + 1, budget + 10, budget + 40, budget + 1700]) {
    const out = reducePoints(parsePointCloud2(cloudMsg(simulatedScene(total))), budget);
    assert.ok(out.count <= budget, `${total} → ${out.count} is over the ceiling`);
    assert.ok(out.count >= budget * 0.85, `${total} → ${out.count} keeps too little of a ${budget} budget`);
  }
});

test('reducing keeps the near wall dense: the budget is not spent on the far points', () => {
  const cloud = parsePointCloud2(cloudMsg(simulatedScene(4010)));
  const out = reducePoints(cloud, 1000);
  assert.ok(out.count <= 1000 && out.count >= 850, `${out.count}`);
  let near = 0;
  for (let i = 0; i < out.count; i++) if (out.xyz[i * 3] < 6) near++;
  assert.ok(near > out.count * 0.5, `only ${near} of ${out.count} points are on the wall`);
});

test('a cloud under the ceiling goes through untouched', () => {
  const cloud = parsePointCloud2(cloudMsg(wall(500)));
  assert.equal(reducePoints(cloud, 1000), cloud);
});

test('the stride is the last resort and the ceiling still holds', () => {
  // every point in its own far-apart voxel until the grid has grown a lot, with a tiny budget
  const cloud = parsePointCloud2(cloudMsg(Array.from({ length: 3000 }, (_, i) => [i * 1000, 0, 0])));
  assert.ok(reducePoints(cloud, 10).count <= 10);
});

test('the frame is self-describing and its floats are aligned', () => {
  const cloud = parsePointCloud2(cloudMsg([[1, 2, 3, 4], [5, 6, 7, 8]]));
  for (const deviceId of ['7', 'uav_12345']) {
    const buf = encodeLidarFrame({ deviceId, level: 'high', cloud });
    assert.equal(buf[0], MSG_TYPE_LIDAR_FRAME);
    assert.equal(buf[1], deviceId.length);
    assert.equal(buf[2], LIDAR_LEVELS.high.id);
    assert.equal(buf.toString('utf8', 4, 4 + deviceId.length), deviceId);
    const padded = Math.ceil(deviceId.length / 4) * 4;
    assert.equal(buf.readUInt32LE(4 + padded), 2);
    const floatsAt = 8 + padded;
    assert.equal(floatsAt % 4, 0, 'a Float32Array view on the client needs a multiple of 4');
    assert.equal(buf.length, floatsAt + 2 * 4 * 4);
    assert.deepEqual(
      [0, 1, 2, 3, 4, 5, 6, 7].map((i) => buf.readFloatLE(floatsAt + i * 4)),
      [1, 2, 3, 4, 5, 6, 7, 8]
    );
  }
});
