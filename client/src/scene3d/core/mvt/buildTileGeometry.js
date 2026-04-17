import * as THREE from 'three';
import earcut from 'earcut';
// based on https://github.com/lorenzoMezza/Three-geo-play

// Each layer gets a distinct Y offset to avoid z-fighting between coplanar meshes.
// Small enough to be invisible from normal camera angles (~cm scale).
const LAYER_CONFIG = {
  //           color       yOffset   allowedClasses (null = all)
  landcover: { color: 0xa8d878, y: 0.0, classes: null },
  landuse: { color: 0xe8ddd0, y: 0.01, classes: new Set(['residential', 'commercial', 'retail']) },
  park: { color: 0x90d060, y: 0.02, classes: null },
  water: { color: 0x60a8e8, y: 0.03, classes: null },
  waterway: { color: 0x60a8e8, y: 0.04, classes: null },
  transportation: { color: 0xffffff, y: 0.06, classes: null },
  building: { color: 0xc8bdb0, y: 0.08, classes: null },
};

const LAYER_ORDER = Object.keys(LAYER_CONFIG);

// Returns signed area of a flat [x,y,...] ring. Positive = clockwise (outer in MVT tile coords).
const signedArea = (ring) => {
  let area = 0;
  for (let i = 0, len = ring.length - 2; i < len; i += 2) {
    area += ring[i] * ring[i + 3] - ring[i + 2] * ring[i + 1];
  }
  return area / 2;
};

// Groups flat rings into polygons [{outer, holes}] using MVT winding convention:
// outer rings have positive signed area (clockwise), holes have negative (CCW).
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

const buildPolygonGeometry = (rings, extent, tileOriginX, tileOriginZ, tileSize, yOffset) => {
  if (!rings || rings.length === 0) return null;

  const scale = tileSize / extent;
  const polygons = groupRings(rings);
  if (polygons.length === 0) return null;

  const allPositions = [];

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
      allPositions.push(tileOriginX + verts[vi] * scale, yOffset, tileOriginZ + verts[vi + 1] * scale);
    }
  }

  if (allPositions.length === 0) return null;

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(allPositions), 3));
  geo.computeVertexNormals();
  return geo;
};

/**
 * Parses a decoded VectorTile (from @mapbox/vector-tile) and returns
 * an array of { geometry, color, renderOrder } ready to add to the scene.
 */
export const buildTileGeometry = (vectorTile, tileOriginX, tileOriginZ, tileSize) => {
  const meshes = [];

  for (let li = 0; li < LAYER_ORDER.length; li++) {
    const layerName = LAYER_ORDER[li];
    const layer = vectorTile.layers[layerName];
    if (!layer) continue;

    const { color, y: yOffset, classes } = LAYER_CONFIG[layerName];
    const extent = layer.extent || 4096;

    for (let fi = 0; fi < layer.length; fi++) {
      const feature = layer.feature(fi);
      if (classes && !classes.has(feature.properties?.class)) continue;
      if (feature.type !== 3 && feature.type !== 2) continue;

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

        const geo = buildPolygonGeometry(rings, extent, tileOriginX, tileOriginZ, tileSize, yOffset);
        if (geo) meshes.push({ geometry: geo, color, renderOrder: li });
      } else if (feature.type === 2 && (layerName === 'transportation' || layerName === 'waterway')) {
        const width = layerName === 'waterway' ? 1.0 : 1.5;
        for (const ring of rawGeom) {
          if (ring.length < 2) continue;
          const lineGeo = buildLineGeometry(ring, extent, tileOriginX, tileOriginZ, tileSize, width, yOffset);
          if (lineGeo) meshes.push({ geometry: lineGeo, color, renderOrder: li });
        }
      }
    }
  }

  return meshes;
};

/**
 * Converts a polyline (array of {x,y}) to a flat ribbon geometry (road/waterway).
 * width in meters.
 */
const buildLineGeometry = (points, extent, tileOriginX, tileOriginZ, tileSize, width, yOffset) => {
  if (points.length < 2) return null;
  const scale = tileSize / extent;
  const hw = width / 2;

  const positions = [];
  const indices = [];

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

    const base = positions.length / 3;
    positions.push(ax + nx, yOffset, az + nz);
    positions.push(ax - nx, yOffset, az - nz);
    positions.push(bx + nx, yOffset, bz + nz);
    positions.push(bx - nx, yOffset, bz - nz);

    indices.push(base, base + 1, base + 2, base + 1, base + 3, base + 2);
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
  geo.setIndex(indices);
  geo.computeVertexNormals();
  return geo;
};
