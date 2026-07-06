import React from 'react';
import { useSelector } from 'react-redux';

import R3FCanvas from './core/R3FCanvas';
import R3FMission from './scene/R3FMission';
import R3DMarkers from './scene/R3DMarkers';
import R3FDevices from './scene/R3FDevices';
import SelectDevice3D from './scene/SelectDevice3D';
import DownloadYamlButton from './controls/DownloadYamlButton';
import Scene3DNavigationControl from './controls/Scene3DNavigationControl';

const Scene3DCanvas = ({ className, style }) => {
  const routes = useSelector((state) => state.mission.route);
  const sessionMarkers = useSelector((state) => state.session.markers);

  return (
    <div
      className={className}
      style={{ display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0, ...style }}
    >
      <R3FCanvas>
        <R3FMission routes={routes} />
        <R3DMarkers elements={sessionMarkers} />
        <R3FDevices />
        <SelectDevice3D />
      </R3FCanvas>
      <Scene3DNavigationControl />
      <DownloadYamlButton />
    </div>
  );
};

export default Scene3DCanvas;
