import { useId, useEffect } from 'react';
import { useTheme } from '@mui/styles';
import { map } from '../MapView';
import { findFonts } from '../mapUtil';

const MapMarkersCreate = ({ markers, showTitles }) => {
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
    map.on('click', id, (e) => {
      console.log(e);
    });
    map.on('mouseenter', id, () => {
      map.getCanvas().style.cursor = 'pointer';
    });
    map.on('mouseleave', id, () => {
      map.getCanvas().style.cursor = '';
    });
    map.on('click', function (e) {
      // The event object (e) contains information like the
      // coordinates of the point on the map that was clicked.
      console.log('A click event has occurred at ' + e.lngLat);
    });

    return () => {
      if (map.getLayer(id)) {
        map.removeLayer(id);
      }
      if (map.getSource(id)) {
        map.removeSource(id);
      }
    };
  }, [showTitles]);

  function listtoPoints(mylist) {
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
        waypoints.push({ ...items, image: 'base', title: 'b-' + item_index });
      });
    }

    return waypoints;
  }

  useEffect(() => {
    let markersIcons = listtoPoints(markers);
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

export default MapMarkersCreate;
