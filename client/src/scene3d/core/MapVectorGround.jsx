import { useEffect, useMemo, useState } from 'react';
import { useSelector } from 'react-redux';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { VectorTile } from '@mapbox/vector-tile';
import Pbf from 'pbf';
import maplibregl from 'maplibre-gl';
import { buildTileGeometry } from './mvt/buildTileGeometry';
import { calculateDistanceMercatorToMeters } from './convertion';

const ZOOM = 14;
const GRID = 5;
const MARTIN_URL = `http://${window.location.hostname}:8080/tiles`;

// Shared materials — created once, never recreated per mesh or per render.
const MATERIAL_CACHE = new Map();
const getMaterial = (color) => {
  if (!MATERIAL_CACHE.has(color)) {
    MATERIAL_CACHE.set(
      color,
      new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, depthWrite: true }),
    );
  }
  return MATERIAL_CACHE.get(color);
};

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

const tileNWOffset = (tx, ty, z, origin) => {
  const nw = tileToLngLat(tx, ty, z);
  const originMerc = maplibregl.MercatorCoordinate.fromLngLat(
    { lng: origin.lng, lat: origin.lat },
    0,
  );
  const nwMerc = maplibregl.MercatorCoordinate.fromLngLat({ lng: nw.lng, lat: nw.lat }, 0);
  const d = calculateDistanceMercatorToMeters(originMerc, nwMerc);
  return { x: d.x, z: -d.y };
};

const tileSizeMeters = (tx, ty, z) => {
  const nw = tileToLngLat(tx, ty, z);
  const se = tileToLngLat(tx + 1, ty + 1, z);
  const nwM = maplibregl.MercatorCoordinate.fromLngLat({ lng: nw.lng, lat: nw.lat }, 0);
  const seM = maplibregl.MercatorCoordinate.fromLngLat({ lng: se.lng, lat: se.lat }, 0);
  const d = calculateDistanceMercatorToMeters(nwM, seM);
  return Math.abs(d.x);
};

const fetchTile = async (z, x, y, signal) => {
  const url = `${MARTIN_URL}/${z}/${x}/${y}`;
  const res = await fetch(url, { signal });
  if (!res.ok) throw new Error(`Tile ${z}/${x}/${y} → HTTP ${res.status}`);
  const buf = await res.arrayBuffer();
  return new VectorTile(new Pbf(buf));
};

// ── per-tile mesh group ────────────────────────────────────────────────────

const TileGroup = ({ tx, ty, originLng, originLat }) => {
  const [meshes, setMeshes] = useState([]);
  const { invalidate } = useThree();

  useEffect(() => {
    const ctrl = new AbortController();
    const origin = { lng: originLng, lat: originLat };

    (async () => {
      try {
        const tile = await fetchTile(ZOOM, tx, ty, ctrl.signal);
        const nw = tileNWOffset(tx, ty, ZOOM, origin);
        const size = tileSizeMeters(tx, ty, ZOOM);
        const built = buildTileGeometry(tile, nw.x, nw.z, size);
        if (!ctrl.signal.aborted) {
          setMeshes(built);
          invalidate();
        }
      } catch (e) {
        if (e.name !== 'AbortError') console.warn(`[MapVectorGround] tile ${tx}/${ty}:`, e.message);
      }
    })();

    return () => {
      ctrl.abort();
      setMeshes((prev) => {
        prev.forEach((m) => m.geometry?.dispose());
        return [];
      });
    };
  }, [tx, ty, originLng, originLat]);

  return (
    <group>
      {meshes.map((m, i) => (
        <mesh
          key={i}
          geometry={m.geometry}
          material={getMaterial(m.color)}
          renderOrder={m.renderOrder}
        />
      ))}
    </group>
  );
};

// ── root component ─────────────────────────────────────────────────────────

const MapVectorGround = () => {
  const originLng = useSelector((state) => state.session.scene3d.origin?.lng);
  const originLat = useSelector((state) => state.session.scene3d.origin?.lat);

  const tiles = useMemo(() => {
    if (originLng == null || originLat == null) return null;
    const center = lngLatToTile(originLng, originLat, ZOOM);
    const half = Math.floor(GRID / 2);
    const result = [];
    for (let dy = -half; dy <= half; dy++) {
      for (let dx = -half; dx <= half; dx++) {
        const tx = center.x + dx;
        const ty = center.y + dy;
        result.push({ tx, ty, key: `${tx}-${ty}` });
      }
    }
    return result;
  }, [originLng, originLat]);

  if (!tiles) return null;

  return (
    <>
      {tiles.map(({ tx, ty, key }) => (
        <TileGroup key={key} tx={tx} ty={ty} originLng={originLng} originLat={originLat} />
      ))}
    </>
  );
};

export default MapVectorGround;
