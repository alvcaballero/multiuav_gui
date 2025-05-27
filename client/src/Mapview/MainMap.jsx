import React, { useState, useCallback, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';

import MapView from './MapView';
import MapMissions from './MapMissions';
import MapMarkers from './MapMarkers';
import MapElements from './MapElements';
import MapPositions from './MapPositions';
import MapSelectedDevice from './MapSelectedDevice';
import MapScale from './MapScale';
import { devicesActions } from '../store';
import MapDefaultCamera from './MapDefaultCamera';
import MapLiveRoutes from './MapLiveRoutes';
import MapGeocoder from './geocoder/MapGeocoder';
import MapGeofence from './MapGeofence';

const MainMap = ({
  filteredPositions,
  markers = [],
  selectedPosition,
  filteredMissiondeviceid = -1,
  routes = [],
  onEventsClick,
}) => {
  const dispatch = useDispatch();
  const onMarkerClick = useCallback(
    (_, deviceId) => {
      dispatch(devicesActions.selectId(deviceId));
    },
    [dispatch]
  );
  return (
    <>
      <MapView>
        <MapMarkers markers={markers} />
        <MapMissions filtereddeviceid={filteredMissiondeviceid} routes={routes} />
        <MapElements />
        <MapGeofence/>
        <MapLiveRoutes/>
        <MapPositions
          positions={filteredPositions}
          onClick={onMarkerClick}
          selectedPosition={selectedPosition}
          showStatus
        />
        <MapSelectedDevice />
        <MapDefaultCamera />
      </MapView>
      <MapScale />
      <MapGeocoder />
    </>
  );
};
export default MainMap;
