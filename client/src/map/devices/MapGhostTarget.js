import { useId, useEffect } from 'react';
import { useTheme } from '@mui/material';
import { map } from '../core/mapInstance';
import { findFonts } from '../core/mapUtil';

const EMPTY = { type: 'FeatureCollection', features: [] };
const HEADING_TICK_M = 6; // length of the heading tick drawn from the target
const EARTH_RADIUS = 6378137;

const toRad = (deg) => (deg * Math.PI) / 180;

const offsetPoint = ([lon, lat], bearing, meters) => {
  const b = toRad(bearing);
  const dLat = (meters * Math.cos(b)) / EARTH_RADIUS;
  const dLon = (meters * Math.sin(b)) / (EARTH_RADIUS * Math.cos(toRad(lat)));
  return [lon + (dLon * 180) / Math.PI, lat + (dLat * 180) / Math.PI];
};

/**
 * Pending manual move: dashed line from the UAV to where it would end up, the target point,
 * a tick pointing to the target heading and a label. Renders nothing while `to` is null.
 * @param {[number, number]} from [lon, lat] of the UAV
 * @param {[number, number]|null} to [lon, lat] of the target
 * @param {number} course target heading, degrees from north
 * @param {string} label
 */
const MapGhostTarget = ({ from, to, course = 0, label = '' }) => {
  const id = useId();
  const theme = useTheme();
  const color = theme.palette.primary.main;

  useEffect(() => {
    map.addSource(id, { type: 'geojson', data: EMPTY });
    map.addLayer({
      id: `${id}-line`,
      type: 'line',
      source: id,
      filter: ['==', ['get', 'kind'], 'path'],
      paint: { 'line-color': color, 'line-width': 2, 'line-dasharray': [2, 2] },
    });
    map.addLayer({
      id: `${id}-heading`,
      type: 'line',
      source: id,
      filter: ['==', ['get', 'kind'], 'heading'],
      paint: { 'line-color': color, 'line-width': 3 },
    });
    map.addLayer({
      id: `${id}-point`,
      type: 'circle',
      source: id,
      filter: ['==', ['get', 'kind'], 'target'],
      paint: {
        'circle-radius': 9,
        'circle-color': color,
        'circle-opacity': 0.25,
        'circle-stroke-color': color,
        'circle-stroke-width': 2,
      },
    });
    map.addLayer({
      id: `${id}-label`,
      type: 'symbol',
      source: id,
      filter: ['==', ['get', 'kind'], 'target'],
      layout: {
        'text-field': ['get', 'label'],
        'text-font': findFonts(map),
        'text-size': 12,
        'text-anchor': 'top',
        'text-offset': [0, 1.2],
        'text-allow-overlap': true,
      },
      paint: { 'text-color': color, 'text-halo-color': 'white', 'text-halo-width': 1 },
    });
    return () => {
      ['label', 'point', 'heading', 'line'].forEach((suffix) => {
        if (map.getLayer(`${id}-${suffix}`)) map.removeLayer(`${id}-${suffix}`);
      });
      if (map.getSource(id)) map.removeSource(id);
    };
  }, [id, color]);

  useEffect(() => {
    const source = map.getSource(id);
    if (!source) return;
    if (!from || !to) {
      source.setData(EMPTY);
      return;
    }
    source.setData({
      type: 'FeatureCollection',
      features: [
        {
          type: 'Feature',
          geometry: { type: 'LineString', coordinates: [from, to] },
          properties: { kind: 'path' },
        },
        {
          type: 'Feature',
          geometry: {
            type: 'LineString',
            coordinates: [to, offsetPoint(to, course, HEADING_TICK_M)],
          },
          properties: { kind: 'heading' },
        },
        {
          type: 'Feature',
          geometry: { type: 'Point', coordinates: to },
          properties: { kind: 'target', label },
        },
      ],
    });
  }, [id, from, to, course, label]);

  return null;
};

export default MapGhostTarget;
