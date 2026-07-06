import React, { useCallback } from 'react';
import { useDispatch } from 'react-redux';

import MapView from './core/MapView';
import MapMissions from './mission/MapMissions';
import MapMarkers from './environment/MapMarkers';
import MapElements from './environment/MapElements';
import MapPositions from './devices/MapPositions';
import MapSelectedDevice from './devices/MapSelectedDevice';
import MapMissionHome from './mission/MapMissionHome';
import MapScale from './controls/MapScale';
import { devicesActions } from '../store';
import MapDefaultCamera from './controls/MapDefaultCamera';
import MapLiveRoutes from './devices/MapLiveRoutes';
import MapGeocoder from './geocoder/MapGeocoder';
import MapGeofence from './environment/MapGeofence';
import PegmanControl from './PegmanControl/PegmanControl';
import MapObstacles from './environment/MapObstacles';
const MainMap = ({
  filteredPositions,
  markers = [],
  selectedPosition,
  filteredMissiondeviceid = -1,
  routes = [],
}) => {
  const dispatch = useDispatch();
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
        <MapMissions filtereddeviceid={filteredMissiondeviceid} routes={routes} />
        {false && <MapObstacles />}
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
