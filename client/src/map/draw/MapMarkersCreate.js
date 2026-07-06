import { useId, useEffect, useRef, useState, useCallback } from 'react';
import { map } from '../core/MapView';
import { findFonts } from '../core/mapUtil';
import palette from '../../shared/palette';

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
  const basesSourceId = `${id}-bases`;
  const elementsSourceId = `${id}-elements`;
  const linesMarkersId = `${id}-lines`;
  const selectMarkersId = `${id}-select`;

  const iconScale = 0.8;
  const iconPointScale = 0.6;

  const [testkeepValue] = useState(new keepMarkers());

  const setLocationsRef = useRef(setLocations);
  const setMarkersRef = useRef(setMarkers);
  useEffect(() => {
    setLocationsRef.current = setLocations;
  }, [setLocations]);
  useEffect(() => {
    setMarkersRef.current = setMarkers;
  }, [setMarkers]);

  const onMouseEnter = useCallback(() => (map.getCanvas().style.cursor = 'move'), []);
  const onMouseEnterPointer = useCallback(() => (map.getCanvas().style.cursor = 'pointer'), []);
  const onMouseLeave = useCallback(() => (map.getCanvas().style.cursor = ''), []);

  const onMouseClick = useCallback((e) => {
    if (e.hasOwnProperty('features')) {
      if (e.features[0].properties.type === 'element') {
        setLocationsRef.current({ ...e.features[0].properties, type: 'object' });
      }
    } else {
      setLocationsRef.current({
        latitude: e.lngLat.lat,
        longitude: e.lngLat.lng,
        groupId: 0,
        id: 0,
        type: 'point',
      });
    }
  }, []);

  const onMove = useCallback(
    (e) => {
      map.getCanvas().style.cursor = 'grabbing';

      let auxMarkers = testkeepValue.getMarkers();
      let auxselectpoint = testkeepValue.getSelect();

      if (auxselectpoint.id >= 0) {
        if (auxselectpoint.type === 'base') {
          auxMarkers.bases[auxselectpoint.id].latitude = e.lngLat.lat;
          auxMarkers.bases[auxselectpoint.id].longitude = e.lngLat.lng;
          map.getSource(basesSourceId)?.setData({
            type: 'FeatureCollection',
            features: basesToFeatures(auxMarkers.bases),
          });
        } else if (auxselectpoint.type === 'element') {
          auxMarkers.elements[auxselectpoint.groupId].items[auxselectpoint.id].latitude =
            e.lngLat.lat;
          auxMarkers.elements[auxselectpoint.groupId].items[auxselectpoint.id].longitude =
            e.lngLat.lng;
          map.getSource(elementsSourceId)?.setData({
            type: 'FeatureCollection',
            features: elementsToFeatures(auxMarkers.elements),
          });
        }
      }
    },
    [testkeepValue, basesSourceId, elementsSourceId],
  );

  const onUp = useCallback(
    (e) => {
      map.getCanvas().style.cursor = '';

      let auxMarkers = testkeepValue.getMarkers();
      let auxselectpoint = testkeepValue.getSelect();

      if (auxselectpoint.id >= 0) {
        if (auxselectpoint.type === 'base') {
          auxMarkers.bases[auxselectpoint.id].latitude = e.lngLat.lat;
          auxMarkers.bases[auxselectpoint.id].longitude = e.lngLat.lng;
        } else if (auxselectpoint.type === 'element') {
          auxMarkers.elements[auxselectpoint.groupId].items[auxselectpoint.id].latitude =
            e.lngLat.lat;
          auxMarkers.elements[auxselectpoint.groupId].items[auxselectpoint.id].longitude =
            e.lngLat.lng;
        }
      }

      testkeepValue.setSelect({ id: -1 });
      setMarkersRef.current(auxMarkers);

      map.off('mousemove', onMove);
      map.off('touchmove', onMove);
    },
    [testkeepValue, onMove],
  );

  const onMouseDown = useCallback(
    (e) => {
      e.preventDefault();
      testkeepValue.setSelect(e.features[0].properties);
      map.getCanvas().style.cursor = 'grab';
      map.on('mousemove', onMove);
      map.once('mouseup', onUp);
    },
    [testkeepValue, onMove, onUp],
  );

  const onMouseTouchStart = useCallback(
    (e) => {
      if (e.points.length !== 1) return;
      e.preventDefault();
      map.on('touchmove', onMove);
      map.once('touchend', onUp);
    },
    [onMove, onUp],
  );

  function basesToFeatures(bases) {
    return (bases || []).map((base, index) => ({
      type: 'Feature',
      geometry: {
        type: 'Point',
        coordinates: [base.longitude, base.latitude],
      },
      properties: {
        ...base,
        type: 'base',
        groupId: 0,
        id: index,
        image: 'base',
        title: base.name || `b-${index}`,
      },
    }));
  }

  function elementsToFeatures(elements) {
    return (elements || []).flatMap((group, groupIdx) =>
      group.items.map((item, itemIdx) => ({
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [item.longitude, item.latitude],
        },
        properties: {
          ...item,
          type: 'element',
          groupId: groupIdx,
          id: itemIdx,
          image: group.type,
          title: item.name || `${groupIdx}-${itemIdx}`,
        },
      })),
    );
  }

  function markerstolines(item, index) {
    let waypoint_pos = Object.values(item.items).map((it) => [it['longitude'], it['latitude']]);
    return {
      id: item.id,
      type: 'Feature',
      geometry: {
        type: 'LineString',
        coordinates: waypoint_pos,
      },
      properties: {
        name: item.name,
        color: palette.colors_devices[index % 7],
      },
    };
  }

  function selectToPoints(myList) {
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

  const addSymbolLayer = useCallback(
    (layerId, sourceId) => {
      if (showTitles) {
        map.addLayer({
          id: layerId,
          type: 'symbol',
          source: sourceId,
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
          id: layerId,
          type: 'symbol',
          source: sourceId,
          layout: {
            'icon-image': '{image}',
            'icon-size': iconScale,
            'icon-allow-overlap': true,
          },
        });
      }
    },
    [showTitles],
  );

  useEffect(() => {
    map.addSource(basesSourceId, {
      type: 'geojson',
      data: { type: 'FeatureCollection', features: [] },
    });
    map.addSource(elementsSourceId, {
      type: 'geojson',
      data: { type: 'FeatureCollection', features: [] },
    });
    map.addSource(linesMarkersId, {
      type: 'geojson',
      data: { type: 'FeatureCollection', features: [] },
    });
    map.addSource(selectMarkersId, {
      type: 'geojson',
      data: { type: 'FeatureCollection', features: [] },
    });

    map.addLayer({
      id: selectMarkersId,
      type: 'symbol',
      source: selectMarkersId,
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
        id: linesMarkersId,
        source: linesMarkersId,
        type: 'line',
        paint: {
          'line-color': ['get', 'color'],
          'line-width': 2,
        },
      });
    }

    addSymbolLayer(elementsSourceId, elementsSourceId);
    addSymbolLayer(basesSourceId, basesSourceId);

    if (SelectItems) {
      map.on('mouseenter', elementsSourceId, onMouseEnterPointer);
      map.on('mouseleave', elementsSourceId, onMouseLeave);
      map.on('click', elementsSourceId, onMouseClick);
      map.on('mouseenter', basesSourceId, onMouseEnterPointer);
      map.on('mouseleave', basesSourceId, onMouseLeave);
      map.on('click', basesSourceId, onMouseClick);
    }
    if (CreateItems) {
      map.on('click', onMouseClick);
    }
    if (moveMarkers) {
      map.on('mouseenter', basesSourceId, onMouseEnter);
      map.on('mouseleave', basesSourceId, onMouseLeave);
      map.on('mousedown', basesSourceId, onMouseDown);
      map.on('touchstart', basesSourceId, onMouseTouchStart);
      map.on('mouseenter', elementsSourceId, onMouseEnter);
      map.on('mouseleave', elementsSourceId, onMouseLeave);
      map.on('mousedown', elementsSourceId, onMouseDown);
      map.on('touchstart', elementsSourceId, onMouseTouchStart);
    }

    return () => {
      map.off('click', onMouseClick);

      map.off('click', elementsSourceId, onMouseClick);
      map.off('mouseenter', elementsSourceId, onMouseEnterPointer);
      map.off('mouseenter', elementsSourceId, onMouseEnter);
      map.off('mouseleave', elementsSourceId, onMouseLeave);
      map.off('mousedown', elementsSourceId, onMouseDown);
      map.off('touchstart', elementsSourceId, onMouseTouchStart);

      map.off('click', basesSourceId, onMouseClick);
      map.off('mouseenter', basesSourceId, onMouseEnterPointer);
      map.off('mouseenter', basesSourceId, onMouseEnter);
      map.off('mouseleave', basesSourceId, onMouseLeave);
      map.off('mousedown', basesSourceId, onMouseDown);
      map.off('touchstart', basesSourceId, onMouseTouchStart);

      [selectMarkersId, linesMarkersId, elementsSourceId, basesSourceId].forEach((layerId) => {
        if (map.getLayer(layerId)) map.removeLayer(layerId);
      });
      [selectMarkersId, linesMarkersId, elementsSourceId, basesSourceId].forEach((sourceId) => {
        if (map.getSource(sourceId)) map.removeSource(sourceId);
      });
    };
  }, [
    showTitles,
    showLines,
    moveMarkers,
    SelectItems,
    CreateItems,
    addSymbolLayer,
    basesSourceId,
    elementsSourceId,
    linesMarkersId,
    selectMarkersId,
    onMouseDown,
    onMouseTouchStart,
    onMouseEnter,
    onMouseEnterPointer,
    onMouseLeave,
    onMouseClick,
  ]);

  useEffect(() => {
    testkeepValue.initMarkers(markers);

    map.getSource(basesSourceId)?.setData({
      type: 'FeatureCollection',
      features: basesToFeatures(markers.bases),
    });

    map.getSource(elementsSourceId)?.setData({
      type: 'FeatureCollection',
      features: elementsToFeatures(markers.elements),
    });

    map.getSource(linesMarkersId)?.setData({
      type: 'FeatureCollection',
      features: (markers.elements || []).map(markerstolines),
    });
  }, [
    markers,
    showTitles,
    showLines,
    moveMarkers,
    SelectItems,
    CreateItems,
    testkeepValue,
    basesSourceId,
    elementsSourceId,
    linesMarkersId,
  ]);

  useEffect(() => {
    const selectPoints = selectToPoints(selectMarkers);
    map.getSource(selectMarkersId)?.setData({
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
  }, [
    selectMarkers,
    showTitles,
    showLines,
    moveMarkers,
    SelectItems,
    CreateItems,
    selectMarkersId,
  ]);

  return null;
};

export default MapMarkersCreate;
