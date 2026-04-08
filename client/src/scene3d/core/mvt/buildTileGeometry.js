import * as THREE from 'three';
import earcut from 'earcut';

// Each layer gets a distinct Y offset to avoid z-fighting between coplanar meshes.
// Small enough to be invisible from normal camera angles (~cm scale).
const LAYER_CONFIG = {
  //           color       yOffset
  landcover:  { color: 0xd0e8c0, y: 0.00 },
  landuse:    { color: 0xf0ece4, y: 0.01 },
  park:       { color: 0xd8ecbc, y: 0.02 },
  water:      { color: 0xa0c8f0, y: 0.03 },
  waterway:   { color: 0xa0c8f0, y: 0.04 },
  transportation: { color: 0xffffff, y: 0.06 },
  building:   { color: 0xe0d8d0, y: 0.08 },
};

const LAYER_ORDER = Object.keys(LAYER_CONFIG);

/**
 * Builds a THREE.BufferGeometry from polygon rings (flat [x,y,...] arrays)
 * using earcut triangulation. yOffset separates layers to avoid z-fighting.
 */
const buildPolygonGeometry = (rings, extent, tileOriginX, tileOriginZ, tileSize, yOffset) => {
  if (!rings || rings.length === 0) return null;

  const scale = tileSize / extent;
  const verts = [];
  const holeIndices = [];

  for (let r = 0; r < rings.length; r++) {
    if (r > 0) holeIndices.push(verts.length / 2);
    const ring = rings[r];
    for (let i = 0; i < ring.length - 2; i += 2) {
      verts.push(ring[i], ring[i + 1]);
    }
  }

  const triangles = earcut(verts, holeIndices.length ? holeIndices : null, 2);
  if (!triangles || triangles.length === 0) return null;

  const positions = new Float32Array(triangles.length * 3);
  for (let i = 0; i < triangles.length; i++) {
    const vi = triangles[i] * 2;
    positions[i * 3]     = tileOriginX + verts[vi]     * scale;
    positions[i * 3 + 1] = yOffset;
    positions[i * 3 + 2] = tileOriginZ + verts[vi + 1] * scale;
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
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

    const { color, y: yOffset } = LAYER_CONFIG[layerName];
    const extent = layer.extent || 4096;

    for (let fi = 0; fi < layer.length; fi++) {
      const feature = layer.feature(fi);
      if (feature.type !== 3 && feature.type !== 2) continue;

      const rawGeom = feature.loadGeometry();

      if (feature.type === 3) {
        const rings = rawGeom.map((ring) => {
          const flat = new Array(ring.length * 2);
          for (let i = 0; i < ring.length; i++) {
            flat[i * 2]     = ring[i].x;
            flat[i * 2 + 1] = ring[i].y;
          }
          return flat;
        });

        const geo = buildPolygonGeometry(rings, extent, tileOriginX, tileOriginZ, tileSize, yOffset);
        if (geo) meshes.push({ geometry: geo, color, renderOrder: li });

      } else if (feature.type === 2 && layerName === 'transportation') {
        for (const ring of rawGeom) {
          if (ring.length < 2) continue;
          const lineGeo = buildLineGeometry(ring, extent, tileOriginX, tileOriginZ, tileSize, 1.5, yOffset);
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
    const ax = tileOriginX + points[i].x     * scale;
    const az = tileOriginZ + points[i].y     * scale;
    const bx = tileOriginX + points[i + 1].x * scale;
    const bz = tileOriginZ + points[i + 1].y * scale;

    const dx = bx - ax;
    const dz = bz - az;
    const len = Math.sqrt(dx * dx + dz * dz) || 1;
    const nx = (-dz / len) * hw;
    const nz = (dx  / len) * hw;

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
