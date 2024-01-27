import React, { useState, useCallback, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';

import MapView from './MapView';
import MapMissions from './MapMissions';
import MapMarkers from './MapMarkers';
import MapPositions from './MapPositions';
import MapSelectedDevice from './MapSelectedDevice';
import MapScale from './MapScale';
import { devicesActions } from '../store';

const MainMap = ({ filteredPositions, markers = [], selectedPosition, onEventsClick }) => {
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
        <MapMissions />

        <MapPositions
          positions={filteredPositions}
          onClick={onMarkerClick}
          selectedPosition={selectedPosition}
          showStatus
        />
        <MapSelectedDevice />
      </MapView>
      <MapScale />
    </>
  );
};
export default MainMap;
