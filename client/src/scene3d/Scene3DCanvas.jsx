import React from 'react';
import { useSelector } from 'react-redux';

import R3FCanvas from './core/R3FCanvas';
import R3FMission from './scene/R3FMission';
import R3DMarkers from './scene/R3DMarkers';
import R3FDevices from './scene/R3FDevices';
import SelectDevice3D from './scene/SelectDevice3D';
import DownloadYamlButton from './controls/DownloadYamlButton';

const Scene3DCanvas = ({ className }) => {
  const routes = useSelector((state) => state.mission.route);
  const sessionMarkers = useSelector((state) => state.session.markers);

  return (
    <div className={className}>
      <R3FCanvas>
        <R3FMission routes={routes} />
        <R3DMarkers elements={sessionMarkers} />
        <R3FDevices />
        <SelectDevice3D />
      </R3FCanvas>
      <DownloadYamlButton />
    </div>
  );
};

export default Scene3DCanvas;
