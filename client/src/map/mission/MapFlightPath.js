import { useId, useEffect } from 'react';

import { map } from '../core/mapInstance';
import palette from '../../shared/palette';

const EMPTY_TRACKS = [];
const deviceColorCount = Object.keys(palette.colors_devices).length;

// Draws the actually-flown path (from recorded position history) as a dashed
// line, distinct from the planned mission line drawn by MapMissions.
const MapFlightPath = ({ tracks = EMPTY_TRACKS }) => {
  const id = useId();

  useEffect(() => {
    map.addSource(id, {
      type: 'geojson',
      data: { type: 'FeatureCollection', features: [] },
    });
    map.addLayer({
      id,
      source: id,
      type: 'line',
      layout: { 'line-join': 'round', 'line-cap': 'round' },
      paint: {
        'line-color': ['get', 'color'],
        'line-width': 3,
        'line-dasharray': [2, 1.5],
      },
    });

    return () => {
      if (map.getLayer(id)) {
        map.removeLayer(id);
      }
      if (map.getSource(id)) {
        map.removeSource(id);
      }
    };
  }, [id]);

  useEffect(() => {
    map.getSource(id)?.setData({
      type: 'FeatureCollection',
      features: tracks.map((track) => ({
        type: 'Feature',
        geometry: { type: 'LineString', coordinates: track.points },
        properties: {
          color: palette.colors_devices[track.deviceId % deviceColorCount],
        },
      })),
    });
  }, [tracks, id]);

  return null;
};

export default MapFlightPath;
