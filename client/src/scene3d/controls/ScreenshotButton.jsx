import React from 'react';
import { IconButton, Tooltip } from '@mui/material';
import PhotoCameraIcon from '@mui/icons-material/PhotoCamera';

const ScreenshotButton = () => {
  const handleScreenshot = () => {
    window.dispatchEvent(new CustomEvent('scene-screenshot'));
  };

  return (
    <div
      style={{
        position: 'absolute',
        top: '360px',
        right: '20px',
        zIndex: 10,
        pointerEvents: 'auto',
      }}
    >
      <Tooltip title="Screenshot 3D scene">
        <IconButton
          onClick={handleScreenshot}
          style={{
            backgroundColor: 'white',
            boxShadow: '0 0 0 2px rgba(0,0,0,0.1)',
          }}
        >
          <PhotoCameraIcon color="primary" />
        </IconButton>
      </Tooltip>
    </div>
  );
};

export default ScreenshotButton;
