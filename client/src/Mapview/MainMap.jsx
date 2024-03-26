import React, { useState, useCallback, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';

import MapView from './MapView';
import MapMissions from './MapMissions';
import MapMarkers from './MapMarkers';
import MapPositions from './MapPositions';
import MapSelectedDevice from './MapSelectedDevice';
import MapScale from './MapScale';
import { devicesActions } from '../store';
import MapDefaultCamera from './MapDefaultCamera';

const MainMap = ({
  filteredPositions,
  markers = [],
  selectedPosition,
  filteredMissiondeviceid = -1,
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
        <MapMissions filtereddeviceid={filteredMissiondeviceid} />

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
    </>
  );
};
export default MainMap;
