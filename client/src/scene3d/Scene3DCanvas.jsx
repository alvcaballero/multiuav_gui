import React, { useState } from 'react';
import { useSelector } from 'react-redux';
import { FormControlLabel, Switch } from '@mui/material';

import R3FCanvas from './core/R3FCanvas';
import R3FMission from './scene/R3FMission';
import R3DMarkers from './scene/R3DMarkers';
import R3FDevices from './scene/R3FDevices';
import SelectDevice3D from './scene/SelectDevice3D';
import SceneScreenshot from './scene/SceneScreenshot';
import DownloadYamlButton from './controls/DownloadYamlButton';
import ScreenshotButton from './controls/ScreenshotButton';
import Scene3DNavigationControl from './controls/Scene3DNavigationControl';

const Scene3DCanvas = ({ className, style }) => {
  const routes = useSelector((state) => state.mission.route);
  const sessionMarkers = useSelector((state) => state.session.markers);
  const [showBoundingBoxes, setShowBoundingBoxes] = useState(true);

  return (
    <div
      className={className}
      style={{ display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0, ...style }}
    >
      <R3FCanvas>
        <R3FMission routes={routes} />
        <R3DMarkers elements={sessionMarkers} showBoundingBoxes={showBoundingBoxes} />
        <R3FDevices />
        <SelectDevice3D />
        <SceneScreenshot />
      </R3FCanvas>
      <Scene3DNavigationControl />
      <DownloadYamlButton />
      <ScreenshotButton />
      <FormControlLabel
        sx={{
          position: 'absolute',
          bottom: 8,
          left: 8,
          zIndex: 1,
          backgroundColor: 'background.paper',
          borderRadius: 1,
          px: 1,
          m: 0,
        }}
        control={
          <Switch
            size="small"
            checked={showBoundingBoxes}
            onChange={(e) => setShowBoundingBoxes(e.target.checked)}
          />
        }
        label="Bounding boxes"
      />
    </div>
  );
};

export default Scene3DCanvas;
