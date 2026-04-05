import { useMemo } from 'react';
import { useSelector } from 'react-redux';
import { TextureLoader } from 'three';
import { useLoader } from '@react-three/fiber';
import maplibregl from 'maplibre-gl';
import { calculateDistanceMercatorToMeters } from './convertion';

const ZOOM = 17;
const GRID = 7; // 7x7 tiles centered on origin (~5.3 km at lat 37°)

// Converts lat/lng to OSM tile coordinates at given zoom
const lngLatToTile = (lng, lat, zoom) => {
  const n = Math.pow(2, zoom);
  const x = Math.floor(((lng + 180) / 360) * n);
  const latRad = (lat * Math.PI) / 180;
  const y = Math.floor(((1 - Math.log(Math.tan(latRad) + 1 / Math.cos(latRad)) / Math.PI) / 2) * n);
  return { x, y };
};

// Returns the lat/lng of the NW corner of a tile
const tileToLngLat = (tx, ty, zoom) => {
  const n = Math.pow(2, zoom);
  const lng = (tx / n) * 360 - 180;
  const latRad = Math.atan(Math.sinh(Math.PI * (1 - (2 * ty) / n)));
  const lat = (latRad * 180) / Math.PI;
  return { lng, lat };
};

// Computes the center lat/lng of a tile
const tileCenterLngLat = (tx, ty, zoom) => {
  const nw = tileToLngLat(tx, ty, zoom);
  const se = tileToLngLat(tx + 1, ty + 1, zoom);
  return {
    lng: (nw.lng + se.lng) / 2,
    lat: (nw.lat + se.lat) / 2,
  };
};

// Computes width/height in meters of a tile
const tileSizeMeters = (tx, ty, zoom) => {
  const nw = tileToLngLat(tx, ty, zoom);
  const se = tileToLngLat(tx + 1, ty + 1, zoom);

  const origin = maplibregl.MercatorCoordinate.fromLngLat({ lng: nw.lng, lat: nw.lat }, 0);
  const east = maplibregl.MercatorCoordinate.fromLngLat({ lng: se.lng, lat: nw.lat }, 0);
  const south = maplibregl.MercatorCoordinate.fromLngLat({ lng: nw.lng, lat: se.lat }, 0);

  const dE = calculateDistanceMercatorToMeters(origin, east);
  const dS = calculateDistanceMercatorToMeters(origin, south);

  return { width: Math.abs(dE.x), height: Math.abs(dS.y) };
};

// Computes XZ offset of a tile center relative to scene origin (in meters)
const tileOffsetFromOrigin = (tx, ty, zoom, sceneOrigin) => {
  const center = tileCenterLngLat(tx, ty, zoom);
  const originMerc = maplibregl.MercatorCoordinate.fromLngLat({ lng: sceneOrigin.lng, lat: sceneOrigin.lat }, 0);
  const centerMerc = maplibregl.MercatorCoordinate.fromLngLat({ lng: center.lng, lat: center.lat }, 0);
  const d = calculateDistanceMercatorToMeters(originMerc, centerMerc);
  return { x: d.x, z: -d.y }; // R3F: X=east, Z=-north
};

// Single tile mesh — loads texture via useLoader (cached by URL)
const TileMesh = ({ url, offsetX, offsetZ, width, height }) => {
  const texture = useLoader(TextureLoader, url);
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[offsetX, 0.0, offsetZ]}>
      <planeGeometry args={[width, height]} />
      <meshStandardMaterial map={texture} roughness={0.9} metalness={0.0} />
    </mesh>
  );
};

const MapTileGround = () => {
  const origin = useSelector((state) => state.session.scene3d.origin);

  const tiles = useMemo(() => {
    if (!origin) return [];
    const center = lngLatToTile(origin.lng, origin.lat, ZOOM);
    const half = Math.floor(GRID / 2);
    const result = [];

    for (let dy = -half; dy <= half; dy++) {
      for (let dx = -half; dx <= half; dx++) {
        const tx = center.x + dx;
        const ty = center.y + dy;
        const url = `https://tile.openstreetmap.org/${ZOOM}/${tx}/${ty}.png`;
        const offset = tileOffsetFromOrigin(tx, ty, ZOOM, origin);
        const size = tileSizeMeters(tx, ty, ZOOM);
        result.push({ url, offset, size, key: `${tx}-${ty}` });
      }
    }
    return result;
  }, [origin]);

  return (
    <>
      {tiles.map(({ url, offset, size, key }) => (
        <TileMesh key={key} url={url} offsetX={offset.x} offsetZ={offset.z} width={size.width} height={size.height} />
      ))}
    </>
  );
};

export default MapTileGround;
