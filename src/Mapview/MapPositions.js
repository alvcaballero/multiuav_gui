import { useId, useCallback, useEffect } from 'react';
import { useSelector } from 'react-redux';
import { map } from './MapView';
import { formatTime, getStatusColor } from '../common/formatter';
import { mapIconKey } from './preloadImages';
import { findFonts } from './mapUtil';
//import { useAttributePreference, usePreference } from '../common/preferences';

const MapPositions = ({ positions, onClick, showStatus, selectedPosition, titleField }) => {
  const id = useId();
  const clusters = `${id}-clusters`;
  const direction = `${id}-direction`;
  const mission = `${id}-mission`;

  const devices = useSelector((state) => state.devices.items);
  const routes = useSelector((state) => state.mission.route);

  const iconScale = 1; // useAttributePreference('iconScale', 1);
  const mapCluster = true; //useAttributePreference('mapCluster', true);
  const hours12 = false; //usePreference('twelveHourFormat');
  const directionType = 'all'; //useAttributePreference('mapDirection', 'selected');

  const createFeature = (devices, position, selectedPositionId) => {
    const device = devices[position.deviceId];
    let showDirection;
    switch (directionType) {
      case 'none':
        showDirection = false;
        break;
      case 'all':
        showDirection = true;
        break;
      default:
        showDirection = selectedPositionId === position.id;
        break;
    }
    let thismission = routes.find((element) => element.uav == device.name);
    let missionColor = thismission ? thismission.id : null;
    return {
      id: position.id,
      deviceId: position.deviceId,
      name: device.name,
      fixTime: formatTime(position.fixTime, 'seconds', hours12),
      category: mapIconKey(device.category),
      color: showStatus ? getStatusColor(device.status) : 'neutral',
      rotation: position.course,
      direction: showDirection,
      mission: thismission ? true : false,
      missionColor: missionColor,
    };
  };

  const onMouseEnter = () => (map.getCanvas().style.cursor = 'pointer');
  const onMouseLeave = () => (map.getCanvas().style.cursor = '');

  const onMapClick = useCallback(
    (event) => {
      if (!event.defaultPrevented && onClick) {
        onClick();
      }
    },
    [onClick]
  );

  const onMarkerClick = useCallback(
    (event) => {
      event.preventDefault();
      const feature = event.features[0];
      if (onClick) {
        onClick(feature.properties.id, feature.properties.deviceId);
      }
    },
    [onClick]
  );

  const onClusterClick = useCallback(
    (event) => {
      event.preventDefault();
      const features = map.queryRenderedFeatures(event.point, {
        layers: [clusters],
      });
      const clusterId = features[0].properties.cluster_id;
      map.getSource(id).getClusterExpansionZoom(clusterId, (error, zoom) => {
        if (!error) {
          map.easeTo({
            center: features[0].geometry.coordinates,
            zoom,
          });
        }
      });
    },
    [clusters]
  );

  useEffect(() => {
    map.addSource(id, {
      type: 'geojson',
      data: {
        type: 'FeatureCollection',
        features: [],
      },
      cluster: mapCluster,
      clusterMaxZoom: 14,
      clusterRadius: 50,
    });
    map.addLayer({
      id,
      type: 'symbol',
      source: id,
      filter: ['!has', 'point_count'],
      layout: {
        'icon-image': '{category}-{color}',
        'icon-size': iconScale,
        'icon-allow-overlap': true,
        'text-field': `{${titleField || 'name'}}`,
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
    map.addLayer({
      id: clusters,
      type: 'symbol',
      source: id,
      filter: ['has', 'point_count'],
      layout: {
        'icon-image': 'background',
        'icon-size': iconScale,
        'text-field': '{point_count_abbreviated}',
        'text-font': findFonts(map),
        'text-size': 14,
      },
    });
    map.addLayer({
      id: direction,
      type: 'symbol',
      source: id,
      filter: ['all', ['!has', 'point_count'], ['==', 'direction', true]],
      layout: {
        'icon-image': 'direction',
        'icon-size': iconScale,
        'icon-allow-overlap': true,
        'icon-rotate': ['get', 'rotation'],
        'icon-rotation-alignment': 'map',
      },
    });
    map.addLayer({
      id: mission,
      type: 'symbol',
      source: id,
      filter: ['all', ['!has', 'point_count'], ['==', 'mission', true]],
      layout: {
        'icon-image': 'mission-{missionColor}',
        'icon-size': iconScale,
      },
    });

    map.on('mouseenter', id, onMouseEnter);
    map.on('mouseleave', id, onMouseLeave);
    map.on('mouseenter', clusters, onMouseEnter);
    map.on('mouseleave', clusters, onMouseLeave);
    map.on('click', id, onMarkerClick);
    map.on('click', clusters, onClusterClick);
    map.on('click', onMapClick);

    return () => {
      map.off('mouseenter', id, onMouseEnter);
      map.off('mouseleave', id, onMouseLeave);
      map.off('mouseenter', clusters, onMouseEnter);
      map.off('mouseleave', clusters, onMouseLeave);
      map.off('click', id, onMarkerClick);
      map.off('click', clusters, onClusterClick);
      map.off('click', onMapClick);

      if (map.getLayer(id)) {
        map.removeLayer(id);
      }
      if (map.getLayer(clusters)) {
        map.removeLayer(clusters);
      }
      if (map.getLayer(direction)) {
        map.removeLayer(direction);
      }
      if (map.getLayer(mission)) {
        map.removeLayer(mission);
      }
      if (map.getSource(id)) {
        map.removeSource(id);
      }
    };
  }, [mapCluster, clusters, direction, mission, onMarkerClick, onClusterClick]);

  useEffect(() => {
    map.getSource(id).setData({
      type: 'FeatureCollection',
      features: positions
        .filter((it) => devices.hasOwnProperty(it.deviceId))
        .map((position) => ({
          type: 'Feature',
          geometry: {
            type: 'Point',
            coordinates: [position.longitude, position.latitude],
          },
          properties: createFeature(devices, position, selectedPosition && selectedPosition.id),
        })),
    });
  }, [devices, positions, selectedPosition]);

  return null;
};

export default MapPositions;
