import { useId, useEffect, useState } from 'react';
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
  selectMarkers = [],
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
  const idselectMarkers = `${id}-select`;

  const iconScale = 0.8;
  const iconPointScale = 0.6;

  const [testkeepValue, settestkeepValue] = useState(new keepMarkers());

  const onMouseEnter = () => (map.getCanvas().style.cursor = 'move');
  const onMouseEnterPointer = () => (map.getCanvas().style.cursor = 'pointer');
  const onMouseLeave = () => (map.getCanvas().style.cursor = '');
  const onMouseClick = (e) => {
    if (e.hasOwnProperty('features')) {
      //console.log(e.features[0]);
      if (e.features[0].properties.type == 'element') {
        setLocations({ ...e.features[0].properties, type: 'object' });
      }
    } else {
      setLocations({
        latitude: e.lngLat.lat,
        longitude: e.lngLat.lng,
        groupId: 0,
        id: 0,
        type: 'point',
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
        auxMarkers.elements[auxselectpoint.groupId]['items'][auxselectpoint.id]['latitude'] = e.lngLat.lat;
        auxMarkers.elements[auxselectpoint.groupId]['items'][auxselectpoint.id]['longitude'] = e.lngLat.lng;
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
        auxMarkers.elements[auxselectpoint.groupId]['items'][auxselectpoint.id]['latitude'] = e.lngLat.lat;
        auxMarkers.elements[auxselectpoint.groupId]['items'][auxselectpoint.id]['longitude'] = e.lngLat.lng;
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
    map.addSource(idselectMarkers, {
      type: 'geojson',
      data: {
        type: 'FeatureCollection',
        features: [],
      },
    });
    map.addLayer({
      id: idselectMarkers,
      type: 'symbol',
      source: idselectMarkers,
      filter: ['!has', 'point_count'],
      layout: {
        'icon-image': 'background-{groupId}',
        'icon-size': iconPointScale,
        'icon-allow-overlap': true,
        'text-allow-overlap': true,
        'text-field': '{title}',
        'text-font': findFonts(map),
        'text-size': 14,
      },
      paint: {
        'text-color': 'white',
      },
    });
    if (showLines) {
      map.addLayer({
        source: linesMarkers,
        id: linesMarkers,
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
      if (map.getLayer(idselectMarkers)) {
        map.removeLayer(idselectMarkers);
      }
      if (map.getLayer(linesMarkers)) {
        map.removeLayer(linesMarkers);
      }
      if (map.getSource(id)) {
        map.removeSource(id);
      }
      if (map.getSource(linesMarkers)) {
        map.removeSource(linesMarkers);
      }
      if (map.getSource(idselectMarkers)) {
        map.removeSource(idselectMarkers);
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
        color: palette.colors_devices[index % 7],
      },
    };
  }
  function selectToPoints(myList) {
    console.log(myList);
    const waypoints = [];
    if (myList.length > 0) {
      myList.forEach((conjunto, index_cj) => {
        conjunto.items.forEach((items, itemIndex) => {
          waypoints.push({
            ...items,
            type: 'element',
            groupId: index_cj % 7,
            id: itemIndex,
            image: conjunto.type,
            title: `${index_cj}-${itemIndex}`,
          });
        });
      });
    }

    return waypoints;
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
  }, [markers, showTitles, showLines, moveMarkers, SelectItems, CreateItems]);

  useEffect(() => {
    let selectPoints = selectToPoints(selectMarkers);
    map.getSource(idselectMarkers).setData({
      type: 'FeatureCollection',
      features: selectPoints.map((point) => ({
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [point.longitude, point.latitude],
        },
        properties: { ...point },
      })),
    });
  }, [selectMarkers, showTitles, showLines, moveMarkers, SelectItems, CreateItems]);

  return null;
};

export default MapMarkersCreate;
