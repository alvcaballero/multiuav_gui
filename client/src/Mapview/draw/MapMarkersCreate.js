import { useId, useEffect, useState } from 'react';
import { useTheme } from '@mui/styles';
import { map } from '../MapView';
import { findFonts } from '../mapUtil';
import palette from '../../common/palette';

class keepMarkers {
  constructor() {
    this.markers = {};
    this.select = { id: -1 };
  }

  initMarkers(value) {
    this.markers = JSON.parse(JSON.stringify(value));
  }

  getMarkers() {
    return this.markers;
  }

  setMarkers(value) {
    this.markers = value;
  }

  getSelect() {
    return this.select;
  }

  setSelect(value) {
    this.select = value;
  }
}

const MapMarkersCreate = ({
  markers,
  showTitles,
  showLines,
  moveMarkers,
  setMarkers,
  SelectItems,
  CreateItems,
  setLocations,
}) => {
  const id = useId();
  const linesMarkers = `${id}-lines`;

  const theme = useTheme();
  const iconScale = 0.4;
  const [testkeepValue, settestkeepValue] = useState(new keepMarkers());

  const onMouseEnter = () => (map.getCanvas().style.cursor = 'move');
  const onMouseEnterPointer = () => (map.getCanvas().style.cursor = 'pointer');
  const onMouseLeave = () => (map.getCanvas().style.cursor = '');
  const onMouseClick = (e) => {
    if (e.hasOwnProperty('features')) {
      //console.log(e.features[0]);
      if (e.features[0].properties.type == 'element') {
        setLocations(e.features[0].properties);
      }
    } else {
      setLocations({
        latitude: e.lngLat.lat,
        longitude: e.lngLat.lng,
        groupId: 0,
        id: 0,
        type: 'select',
      });
    }
  };
  const onMove = (e) => {
    // Set a UI indicator for dragging.
    map.getCanvas().style.cursor = 'grabbing';
    console.log('on move point' + e.lngLat.lng + '-' + e.lngLat.lat);
    // Update the Point feature in `geojson` coordinates
    // and call setData to the source layer `point` on it.
    let auxMarkers = testkeepValue.getMarkers();
    let auxselectpoint = testkeepValue.getSelect();
    console.log(auxMarkers);
    console.log(auxselectpoint);
    if (auxselectpoint.id >= 0) {
      if (auxselectpoint.type == 'base') {
        auxMarkers.bases[auxselectpoint.id]['latitude'] = e.lngLat.lat;
        auxMarkers.bases[auxselectpoint.id]['longitude'] = e.lngLat.lng;
      }
      if (auxselectpoint.type == 'element') {
        auxMarkers.elements[auxselectpoint.groupId]['items'][auxselectpoint.id]['latitude'] =
          e.lngLat.lat;
        auxMarkers.elements[auxselectpoint.groupId]['items'][auxselectpoint.id]['longitude'] =
          e.lngLat.lng;
      }
    }
    let markersIcons = listtoPoints(auxMarkers);
    map.getSource(id)?.setData({
      type: 'FeatureCollection',
      features: markersIcons.map((marker) => ({
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [marker.longitude, marker.latitude],
        },
        properties: { ...marker },
      })),
    });
  };
  const onUp = (e) => {
    let coords = e.lngLat;
    map.getCanvas().style.cursor = '';

    let auxMarkers = testkeepValue.getMarkers();
    let auxselectpoint = testkeepValue.getSelect();
    console.log(auxMarkers);
    console.log(auxselectpoint);
    if (auxselectpoint.id >= 0) {
      if (auxselectpoint.type == 'base') {
        auxMarkers.bases[auxselectpoint.id]['latitude'] = e.lngLat.lat;
        auxMarkers.bases[auxselectpoint.id]['longitude'] = e.lngLat.lng;
      }
      if (auxselectpoint.type == 'element') {
        auxMarkers.elements[auxselectpoint.groupId]['items'][auxselectpoint.id]['latitude'] =
          e.lngLat.lat;
        auxMarkers.elements[auxselectpoint.groupId]['items'][auxselectpoint.id]['longitude'] =
          e.lngLat.lng;
      }
    }
    testkeepValue.getSelect({ id: -1 });
    setMarkers(auxMarkers);

    console.log(`Longitude: ${coords.lng} Latitude: ${coords.lat}`);

    map.off('mousemove', onMove);
    map.off('touchmove', onMove);
  };
  const onMouseDown = (e) => {
    e.preventDefault();

    testkeepValue.setSelect(e.features[0].properties);

    map.getCanvas().style.cursor = 'grab';
    map.on('mousemove', onMove);
    map.once('mouseup', onUp);
  };
  const onMouseTouchStart = (e) => {
    if (e.points.length !== 1) return;
    console.log('touch start');

    // Prevent the default map drag behavior.
    e.preventDefault();

    map.on('touchmove', onMove);
    map.once('touchend', onUp);
  };
  useEffect(() => {
    map.addSource(id, {
      type: 'geojson',
      data: {
        type: 'FeatureCollection',
        features: [],
      },
    });
    map.addSource(linesMarkers, {
      type: 'geojson',
      data: {
        type: 'FeatureCollection',
        features: [],
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
  }, []);

  useEffect(() => {
    if (showLines) {
      map.addLayer({
        source: linesMarkers,
        id: 'markers-line',
        type: 'line',
        paint: {
          'line-color': ['get', 'color'],
          'line-width': 2,
        },
      });
    }

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
    if (SelectItems) {
      map.on('mouseenter', id, onMouseEnterPointer);
      map.on('mouseleave', id, onMouseLeave);
      map.on('click', id, onMouseClick);
    }
    if (CreateItems) {
      map.on('click', onMouseClick);
    }
    if (moveMarkers) {
      map.on('mouseenter', id, onMouseEnter);
      map.on('mouseleave', id, onMouseLeave);
      map.on('mousedown', id, onMouseDown);
      map.on('touchstart', id, onMouseTouchStart);
    }

    return () => {
      map.off('click', onMouseClick);

      map.off('click', id, onMouseClick);
      map.off('mouseenter', id, onMouseEnterPointer);
      map.off('mouseenter', id, onMouseEnter);
      map.off('mouseleave', id, onMouseLeave);
      map.off('mousedown', id, onMouseDown);
      map.off('touchstart', id, onMouseTouchStart);
      if (map.getLayer(id)) {
        map.removeLayer(id);
      }

      if (map.getLayer('markers-line')) {
        map.removeLayer('markers-line');
      }
    };
  }, [showTitles, showLines, moveMarkers, SelectItems, CreateItems]);

  function listtoPoints(myList) {
    const waypoints = [];
    if (myList.elements) {
      myList.elements.forEach((conjunto, index_cj) => {
        conjunto.items.forEach((items, itemIndex) => {
          waypoints.push({
            ...items,
            type: 'element',
            groupId: index_cj,
            id: itemIndex,
            image: conjunto.type,
            title: `${index_cj}-${itemIndex}`,
          });
        });
      });
    }
    if (myList.bases) {
      myList.bases.forEach((items, itemIndex) => {
        waypoints.push({
          ...items,
          type: 'base',
          groupId: 0,
          id: itemIndex,
          image: 'base',
          title: `b-${itemIndex}`,
        });
      });
    }

    return waypoints;
  }

  function markerstolines(item, index) {
    let waypoint_pos = Object.values(item.items).map((it) => [it['longitude'], it['latitude']]);
    //console.log(waypoint_pos);
    return {
      id: item.id,
      type: 'Feature',
      geometry: {
        type: 'LineString',
        coordinates: waypoint_pos,
      },
      properties: {
        name: item.name, // name,
        color: palette.colors_devices[index],
      },
    };
  }

  useEffect(() => {
    testkeepValue.initMarkers(markers);
    let markersIcons = listtoPoints(markers);
    map.getSource(id)?.setData({
      type: 'FeatureCollection',
      features: markersIcons.map((marker) => ({
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [marker.longitude, marker.latitude],
        },
        properties: { ...marker },
      })),
    });

    map.getSource(linesMarkers).setData({
      type: 'FeatureCollection',
      features: markers.elements.map((element, index) => markerstolines(element, index)),
    });
  }, [markers]);

  return null;
};

export default MapMarkersCreate;
