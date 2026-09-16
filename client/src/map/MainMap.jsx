import React, { useCallback, useMemo } from 'react';
import { useDispatch, useSelector } from 'react-redux';

import MapView from './core/MapView';
import { MapMissions } from './mission/MapMissions';
import MapMarkers from './environment/MapMarkers';
import MapElements from './environment/MapElements';
import MapPositions from './devices/MapPositions';
import MapSelectedDevice from './devices/MapSelectedDevice';
import MapMissionHome from './mission/MapMissionHome';
import MapScale from './controls/MapScale';
import { devicesActions } from '../store';
import MapDefaultCamera from './controls/MapDefaultCamera';
import MapLiveRoutes from './devices/MapLiveRoutes';
import MapGeocoder from './controls/MapGeocoder';
import MapGeofence from './environment/MapGeofence';
import PegmanControl from './controls/PegmanControl/PegmanControl';
import MapObstacles from './environment/MapObstacles';
import { getAllInspectionGroups } from '../store/sessionSelectors';
import { useMarkerTypes } from '../hooks/useMarkerTypes';

const EMPTY_MARKERS = [];
const EMPTY_ROUTES = [];

/** ElementGroups/Items (state.session.markers.elements) → MapObstacles' real-data shape. */
const useInspectionObstacles = () => {
  const groups = useSelector(getAllInspectionGroups);
  const { types: markerTypes } = useMarkerTypes();

  return useMemo(
    () =>
      groups.flatMap((group) => {
        const typeGeometry = markerTypes.find((t) => t.id === group.type)?.attributes?.geometry;
        return (group.items || []).flatMap((item) => {
          const geometry = item.attributes?.geometry || typeGeometry;
          if (!geometry || item.latitude == null || item.longitude == null) return [];
          return [
            {
              name: item.name,
              type: group.type,
              latitude: item.latitude,
              longitude: item.longitude,
              geometry_type: geometry.geometry_type,
              dimensions: geometry.dimensions,
              yaw: geometry.yaw || 0,
            },
          ];
        });
      }),
    [groups, markerTypes],
  );
};

const MainMap = ({
  filteredPositions,
  markers = EMPTY_MARKERS,
  selectedPosition,
  filteredMissiondeviceid = -1,
  routes = EMPTY_ROUTES,
}) => {
  const dispatch = useDispatch();
  const obstacles = useInspectionObstacles();
  const onMarkerClick = useCallback(
    (_, deviceId) => {
      dispatch(devicesActions.selectId(deviceId));
    },
    [dispatch],
  );
  return (
    <>
      <MapView>
        <MapMarkers markers={markers} showTitles={true} />
        <MapMissions filteredDeviceId={filteredMissiondeviceid} routes={routes} />
        <MapObstacles obstacles={obstacles} />
        <MapElements />
        <MapGeofence />
        <MapLiveRoutes />
        <MapPositions
          positions={filteredPositions}
          onClick={onMarkerClick}
          selectedPosition={selectedPosition}
          showStatus
        />
        <MapSelectedDevice />
        <MapMissionHome />
        <MapDefaultCamera />
      </MapView>
      <MapScale />
      <MapGeocoder />
      <PegmanControl />
    </>
  );
};
export default MainMap;
