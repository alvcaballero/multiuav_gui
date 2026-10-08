import { useId, useEffect } from 'react';
import turfCircle from '@turf/circle';
import { useTheme } from '@mui/material/styles';
import { map } from '../core/mapInstance';

// Position-accuracy circle per UAV. Ported from traccar-web (src/map/main/MapAccuracy.js).
// `position.accuracy` is in meters; turf wants km.
const MapAccuracy = ({ positions }) => {
  const id = useId();
  const theme = useTheme();

  useEffect(() => {
    map.addSource(id, {
      type: 'geojson',
      data: { type: 'FeatureCollection', features: [] },
    });
    map.addLayer({
      source: id,
      id,
      type: 'fill',
      filter: ['all', ['==', '$type', 'Polygon']],
      metadata: { 'traccar:title': 'PositionAccuracy' },
      paint: {
        'fill-color': theme.palette.geometry.main,
        'fill-outline-color': theme.palette.geometry.main,
        'fill-opacity': 0.25,
      },
    });

    return () => {
      if (map.getLayer(id)) map.removeLayer(id);
      if (map.getSource(id)) map.removeSource(id);
    };
  }, [id, theme.palette.geometry.main]);

  useEffect(() => {
    map.getSource(id)?.setData({
      type: 'FeatureCollection',
      features: positions
        .filter((position) => position.accuracy > 0)
        .map((position) =>
          turfCircle([position.longitude, position.latitude], position.accuracy * 0.001),
        ),
    });
  }, [positions, id]);

  return null;
};

export default MapAccuracy;
