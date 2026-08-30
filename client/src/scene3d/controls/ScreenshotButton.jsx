import React from 'react';
import { IconButton, Tooltip } from '@mui/material';
import PhotoCameraIcon from '@mui/icons-material/PhotoCamera';
import Scene3DControl from './registry/Scene3DControl';
import { controlSurfaceStyle } from './controlStyles';

const ScreenshotButton = () => {
  const handleScreenshot = () => {
    window.dispatchEvent(new CustomEvent('scene-screenshot'));
  };

  return (
    <Scene3DControl corner="top-right">
      <Tooltip title="Screenshot 3D scene">
        <IconButton onClick={handleScreenshot} style={controlSurfaceStyle}>
          <PhotoCameraIcon color="primary" />
        </IconButton>
      </Tooltip>
    </Scene3DControl>
  );
};

export default ScreenshotButton;
