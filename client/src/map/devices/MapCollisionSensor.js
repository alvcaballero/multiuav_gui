import { useId, useEffect, useMemo, useCallback } from 'react';
import { useSelector } from 'react-redux';
import { map } from '../core/mapInstance';

// Proximity arcs around a UAV, one per horizontal collision sensor (front/right/back/left).
// Reads `position.attributes.obstacle_info` = [down, front, right, back, left, up] (meters,
// 100 = no reading) and `position.course` (deg, 0 = north).
//
// The arc radius is drawn in SCREEN pixels (recomputed on zoom), not in real meters: at the
// zoom levels where you see a whole fleet, a 3 m arc would sit underneath the icon.
// The closer the obstacle, the closer the arc to the icon, the more opaque and the redder.

const SIDES = [
  { index: 1, bearing: 0 }, // front
  { index: 2, bearing: 90 }, // right
  { index: 3, bearing: 180 }, // back
  { index: 4, bearing: 270 }, // left
];

const MAX_RANGE = 10; // m — at or beyond this nothing is drawn
const ICON_RADIUS_PX = 12; // arc at distance 0 touches the icon edge
const SPAN_PX = 22; // arc at MAX_RANGE sits this much further out
const THICKNESS_PX = 4;
const HALF_ANGLE = 40; // deg — each arc covers +-HALF_ANGLE around its side
const STEP = 5; // deg between polygon vertices
const EARTH_RADIUS = 6378137;

const toRad = (deg) => (deg * Math.PI) / 180;

const metersPerPixel = (lat, zoom) => (156543.03392 * Math.cos(toRad(lat))) / 2 ** zoom;

/** Point `meters` away from (lon, lat) along `bearing` (deg from north). Flat-earth, fine at < 100 m. */
const offsetPoint = (lon, lat, bearing, meters) => {
  const b = toRad(bearing);
  const dLat = (meters * Math.cos(b)) / EARTH_RADIUS;
  const dLon = (meters * Math.sin(b)) / (EARTH_RADIUS * Math.cos(toRad(lat)));
  return [lon + (dLon * 180) / Math.PI, lat + (dLat * 180) / Math.PI];
};

const arcPolygon = (lon, lat, centerBearing, innerM, outerM) => {
  const angles = [];
  for (let a = -HALF_ANGLE; a <= HALF_ANGLE; a += STEP) angles.push(centerBearing + a);
  const outer = angles.map((a) => offsetPoint(lon, lat, a, outerM));
  const inner = angles.reverse().map((a) => offsetPoint(lon, lat, a, innerM));
  return [[...outer, ...inner, outer[0]]];
};

const buildFeatures = (position, mpp) => {
  const sensors = position?.attributes?.obstacle_info;
  if (!sensors || position.latitude == null || position.longitude == null) return [];
  const course = position.course || 0;

  return SIDES.flatMap(({ index, bearing }) => {
    const distance = sensors[index];
    if (distance == null || distance >= MAX_RANGE) return [];
    const d = Math.max(distance, 0);
    const innerPx = ICON_RADIUS_PX + (d / MAX_RANGE) * SPAN_PX;
    return [
      {
        type: 'Feature',
        geometry: {
          type: 'Polygon',
          coordinates: arcPolygon(
            position.longitude,
            position.latitude,
            course + bearing,
            innerPx * mpp,
            (innerPx + THICKNESS_PX) * mpp,
          ),
        },
        properties: { severity: 1 - d / MAX_RANGE },
      },
    ];
  });
};

/**
 * @param {'selected'|'all'} mode which UAVs get arcs. 'selected' (default) = only the selected one.
 */
const MapCollisionSensor = ({ mode = 'selected' }) => {
  const id = useId();

  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const positions = useSelector((state) => state.session.positions);

  const targets = useMemo(() => {
    if (mode === 'all') return Object.values(positions);
    const selected = positions[selectedDeviceId];
    return selected ? [selected] : [];
  }, [mode, positions, selectedDeviceId]);

  useEffect(() => {
    map.addSource(id, {
      type: 'geojson',
      data: { type: 'FeatureCollection', features: [] },
    });
    map.addLayer({
      source: id,
      id,
      type: 'fill',
      metadata: { 'traccar:title': 'CollisionSensor' },
      paint: {
        'fill-color': [
          'interpolate',
          ['linear'],
          ['get', 'severity'],
          0, '#4caf50',
          0.45, '#ffc107',
          0.8, '#f44336',
        ],
        'fill-opacity': ['interpolate', ['linear'], ['get', 'severity'], 0, 0.15, 1, 0.85],
      },
    });
    return () => {
      if (map.getLayer(id)) map.removeLayer(id);
      if (map.getSource(id)) map.removeSource(id);
    };
  }, [id]);

  const refresh = useCallback(() => {
    const zoom = map.getZoom();
    map.getSource(id)?.setData({
      type: 'FeatureCollection',
      features: targets.flatMap((p) => buildFeatures(p, metersPerPixel(p.latitude ?? 0, zoom))),
    });
  }, [id, targets]);

  useEffect(() => {
    refresh();
    map.on('zoom', refresh);
    return () => map.off('zoom', refresh);
  }, [refresh]);

  return null;
};

export default MapCollisionSensor;
