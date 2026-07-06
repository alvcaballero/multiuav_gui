import * as THREE from 'three';
import earcut from 'earcut';
// based on https://github.com/lorenzoMezza/Three-geo-play

// Each layer gets a distinct Y offset to avoid z-fighting between coplanar meshes.
const LAYER_CONFIG = {
  //           color       yOffset   allowedClasses (null = all)
  landcover: { color: 0xa8d878, y: 0.0, classes: null },
  landuse: { color: 0xe8ddd0, y: 0.01, classes: new Set(['residential', 'commercial', 'retail']) },
  park: { color: 0x90d060, y: 0.02, classes: null },
  water: { color: 0x60a8e8, y: 0.03, classes: null },
  waterway: { color: 0x4488cc, y: 0.04, classes: null },
  transportation: { color: 0xaaaaaa, y: 0.06, classes: null },
  building: { color: 0xc8bdb0, y: 0.08, classes: null },
};

const LAYER_ORDER = Object.keys(LAYER_CONFIG);

const signedArea = (ring) => {
  let area = 0;
  for (let i = 0, len = ring.length - 2; i < len; i += 2) {
    area += ring[i] * ring[i + 3] - ring[i + 2] * ring[i + 1];
  }
  return area / 2;
};

const groupRings = (rings) => {
  const polygons = [];
  let current = null;
  for (const ring of rings) {
    if (signedArea(ring) > 0) {
      current = { outer: ring, holes: [] };
      polygons.push(current);
    } else if (current) {
      current.holes.push(ring);
    }
  }
  return polygons;
};

// Accumulates earcut triangles for all features of a layer into a shared flat array.
const accumulatePolygon = (rings, extent, tileOriginX, tileOriginZ, scale, yOffset, accum) => {
  const polygons = groupRings(rings);

  for (const { outer, holes } of polygons) {
    const verts = [];
    const holeIndices = [];

    const addRing = (ring) => {
      for (let i = 0; i < ring.length - 2; i += 2) verts.push(ring[i], ring[i + 1]);
    };

    addRing(outer);
    for (const hole of holes) {
      holeIndices.push(verts.length / 2);
      addRing(hole);
    }

    const triangles = earcut(verts, holeIndices.length ? holeIndices : null, 2);
    if (!triangles || triangles.length === 0) continue;

    for (let i = 0; i < triangles.length; i++) {
      const vi = triangles[i] * 2;
      accum.push(tileOriginX + verts[vi] * scale, yOffset, tileOriginZ + verts[vi + 1] * scale);
    }
  }
};

// Accumulates ribbon segments for all polylines of a layer into shared flat arrays.
const accumulateLine = (
  points,
  scale,
  tileOriginX,
  tileOriginZ,
  hw,
  yOffset,
  posAccum,
  idxAccum,
) => {
  for (let i = 0; i < points.length - 1; i++) {
    const ax = tileOriginX + points[i].x * scale;
    const az = tileOriginZ + points[i].y * scale;
    const bx = tileOriginX + points[i + 1].x * scale;
    const bz = tileOriginZ + points[i + 1].y * scale;

    const dx = bx - ax;
    const dz = bz - az;
    const len = Math.sqrt(dx * dx + dz * dz) || 1;
    const nx = (-dz / len) * hw;
    const nz = (dx / len) * hw;

    const base = posAccum.length / 3;
    posAccum.push(ax + nx, yOffset, az + nz);
    posAccum.push(ax - nx, yOffset, az - nz);
    posAccum.push(bx + nx, yOffset, bz + nz);
    posAccum.push(bx - nx, yOffset, bz - nz);

    idxAccum.push(base, base + 1, base + 2, base + 1, base + 3, base + 2);
  }
};

const vertsToGeometry = (positions) => {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
  return geo;
};

const indexedVertsToGeometry = (positions, indices) => {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
  geo.setIndex(indices);
  return geo;
};

/**
 * Parses a decoded VectorTile and returns one { geometry, color, renderOrder }
 * per layer that has data — all features within a layer are merged into a single geometry.
 * This reduces draw calls from O(features) to O(layers) per tile.
 */
export const buildTileGeometry = (vectorTile, tileOriginX, tileOriginZ, tileSize) => {
  const meshes = [];

  for (let li = 0; li < LAYER_ORDER.length; li++) {
    const layerName = LAYER_ORDER[li];
    const layer = vectorTile.layers[layerName];
    if (!layer) continue;

    const { color, y: yOffset, classes } = LAYER_CONFIG[layerName];
    const extent = layer.extent || 4096;
    const scale = tileSize / extent;
    const isLine = layerName === 'transportation' || layerName === 'waterway';
    const lineWidth = layerName === 'waterway' ? 3.0 : 6.0;
    const hw = lineWidth / 2;

    // One accumulator per layer — all features merge here.
    const polyAccum = [];
    const linePos = [];
    const lineIdx = [];

    for (let fi = 0; fi < layer.length; fi++) {
      const feature = layer.feature(fi);
      if (classes && !classes.has(feature.properties?.class)) continue;

      const rawGeom = feature.loadGeometry();

      if (feature.type === 3) {
        const rings = rawGeom.map((ring) => {
          const flat = new Array(ring.length * 2);
          for (let i = 0; i < ring.length; i++) {
            flat[i * 2] = ring[i].x;
            flat[i * 2 + 1] = ring[i].y;
          }
          return flat;
        });
        accumulatePolygon(rings, extent, tileOriginX, tileOriginZ, scale, yOffset, polyAccum);
      } else if (feature.type === 2 && isLine) {
        for (const ring of rawGeom) {
          if (ring.length < 2) continue;
          accumulateLine(ring, scale, tileOriginX, tileOriginZ, hw, yOffset, linePos, lineIdx);
        }
      }
    }

    if (polyAccum.length > 0) {
      meshes.push({ geometry: vertsToGeometry(polyAccum), color, renderOrder: li });
    }
    if (linePos.length > 0) {
      meshes.push({ geometry: indexedVertsToGeometry(linePos, lineIdx), color, renderOrder: li });
    }
  }

  return meshes;
};
