import { useEffect, useRef, useState } from 'react';
import { useSelector } from 'react-redux';
import * as THREE from 'three';
import { VectorTile } from '@mapbox/vector-tile';
import Pbf from 'pbf';
import maplibregl from 'maplibre-gl';
import { buildTileGeometry } from './mvt/buildTileGeometry';
import { calculateDistanceMercatorToMeters } from './convertion';

const ZOOM = 14;
const GRID = 5; // 5×5 tiles
const MARTIN_URL = `http://${window.location.hostname}:8080/tiles`;

// ── tile coordinate helpers ────────────────────────────────────────────────

const lngLatToTile = (lng, lat, z) => {
  const n = 2 ** z;
  const x = Math.floor(((lng + 180) / 360) * n);
  const latRad = (lat * Math.PI) / 180;
  const y = Math.floor(((1 - Math.log(Math.tan(latRad) + 1 / Math.cos(latRad)) / Math.PI) / 2) * n);
  return { x, y };
};

const tileToLngLat = (tx, ty, z) => {
  const n = 2 ** z;
  const lng = (tx / n) * 360 - 180;
  const latRad = Math.atan(Math.sinh(Math.PI * (1 - (2 * ty) / n)));
  return { lng, lat: (latRad * 180) / Math.PI };
};

// NW corner of a tile in scene-space meters (X=east, Z=south relative to origin)
const tileNWOffset = (tx, ty, z, origin) => {
  const nw = tileToLngLat(tx, ty, z);
  const originMerc = maplibregl.MercatorCoordinate.fromLngLat({ lng: origin.lng, lat: origin.lat }, 0);
  const nwMerc     = maplibregl.MercatorCoordinate.fromLngLat({ lng: nw.lng,     lat: nw.lat     }, 0);
  const d = calculateDistanceMercatorToMeters(originMerc, nwMerc);
  return { x: d.x, z: -d.y }; // R3F: Z = -north = south
};

const tileSizeMeters = (tx, ty, z) => {
  const nw = tileToLngLat(tx, ty, z);
  const se = tileToLngLat(tx + 1, ty + 1, z);
  const nwM = maplibregl.MercatorCoordinate.fromLngLat({ lng: nw.lng, lat: nw.lat }, 0);
  const seM = maplibregl.MercatorCoordinate.fromLngLat({ lng: se.lng, lat: se.lat }, 0);
  const d = calculateDistanceMercatorToMeters(nwM, seM);
  return Math.abs(d.x); // tiles are ~square; use width
};

// ── fetch one MVT tile ─────────────────────────────────────────────────────

const fetchTile = async (z, x, y, signal) => {
  const url = `${MARTIN_URL}/${z}/${x}/${y}`;
  const res = await fetch(url, { signal });
  if (!res.ok) throw new Error(`Tile ${z}/${x}/${y} → HTTP ${res.status}`);
  const buf = await res.arrayBuffer();
  return new VectorTile(new Pbf(buf));
};

// ── per-tile mesh group ────────────────────────────────────────────────────

const TileGroup = ({ tx, ty, origin }) => {
  const groupRef = useRef();
  const [meshes, setMeshes] = useState([]);

  useEffect(() => {
    if (!origin) return;
    const ctrl = new AbortController();

    (async () => {
      try {
        const tile = await fetchTile(ZOOM, tx, ty, ctrl.signal);
        const nw   = tileNWOffset(tx, ty, ZOOM, origin);
        const size = tileSizeMeters(tx, ty, ZOOM);
        // MVT Y axis grows downward (south), matches our +Z=south convention
        const built = buildTileGeometry(tile, nw.x, nw.z, size);
        if (!ctrl.signal.aborted) setMeshes(built);
      } catch (e) {
        if (e.name !== 'AbortError') console.warn(`[MapVectorGround] tile ${tx}/${ty}:`, e.message);
      }
    })();

    return () => {
      ctrl.abort();
      // dispose geometries on unmount
      setMeshes((prev) => {
        prev.forEach((m) => m.geometry?.dispose());
        return [];
      });
    };
  }, [tx, ty, origin]);

  return (
    <group ref={groupRef}>
      {meshes.map((m, i) => (
        <mesh
          key={i}
          geometry={m.geometry}
          renderOrder={m.renderOrder}
        >
          <meshStandardMaterial
            color={m.color}
            roughness={0.9}
            metalness={0.0}
            side={THREE.DoubleSide}
            depthWrite
          />
        </mesh>
      ))}
    </group>
  );
};

// ── root component ─────────────────────────────────────────────────────────

const MapVectorGround = () => {
  const origin = useSelector((state) => state.session.scene3d.origin);

  if (!origin) return null;

  const center = lngLatToTile(origin.lng, origin.lat, ZOOM);
  const half   = Math.floor(GRID / 2);
  const tiles  = [];

  for (let dy = -half; dy <= half; dy++) {
    for (let dx = -half; dx <= half; dx++) {
      const tx = center.x + dx;
      const ty = center.y + dy;
      tiles.push({ tx, ty, key: `${tx}-${ty}` });
    }
  }

  return (
    <>
      {tiles.map(({ tx, ty, key }) => (
        <TileGroup key={key} tx={tx} ty={ty} origin={origin} />
      ))}
    </>
  );
};

export default MapVectorGround;
