import { useId, useEffect } from 'react';
import { useTheme } from '@mui/styles';
import { map } from './MapView';
import { findFonts } from './mapUtil';

const MapElements = ({ markers, showTitles }) => {
  const id = useId();

  const theme = useTheme();
  const desktop = true;
  const iconScale = 0.4;

  useEffect(() => {
    map.addSource(id, {
      type: 'geojson',
      data: {
        type: 'FeatureCollection',
        features: [],
      },
    });

    if (showTitles) {
      map.addLayer({
        id,
        type: 'symbol',
        source: id,
        filter: ['!has', 'point_count'],
        layout: {
          'icon-image': '{image}',
          'icon-size': iconScale,
          'icon-allow-overlap': true,
          'text-field': '{title}',
          'text-allow-overlap': true,
          'text-anchor': 'bottom',
          'text-offset': [0, -2 * iconScale],
          'text-font': findFonts(map),
          'text-size': 12,
        },
        paint: {
          'text-halo-color': 'white',
          'text-halo-width': 1,
        },
      });
    } else {
      map.addLayer({
        id,
        type: 'symbol',
        source: id,
        layout: {
          'icon-image': '{image}',
          'icon-size': iconScale,
          'icon-allow-overlap': true,
        },
      });
    }

    return () => {
      if (map.getLayer(id)) {
        map.removeLayer(id);
      }
      if (map.getSource(id)) {
        map.removeSource(id);
      }
    };
  }, [showTitles]);

  function list2Points(mylist) {
    const waypoints = [];
    if (mylist.elements) {
      mylist.elements.forEach((conjunto, index_cj) => {
        conjunto.items.forEach((items, item_index) => {
          waypoints.push({ ...items, image: conjunto.type, title: `${index_cj}-${item_index}` });
        });
      });
    }
    if (mylist.bases) {
      mylist.bases.forEach((items, item_index) => {
        waypoints.push({ ...items, image: 'base', title: item_index });
      });
    }

    return waypoints;
  }

  useEffect(() => {
    const markersIcons = list2Points(markers);
    map.getSource(id)?.setData({
      type: 'FeatureCollection',
      features: markersIcons.map(({ latitude, longitude, image, title }) => ({
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [longitude, latitude],
        },
        properties: {
          image: image || 'default-neutral',
          title: title || '',
        },
      })),
    });
  }, [showTitles, markers]);

  return null;
};

export default MapElements;
