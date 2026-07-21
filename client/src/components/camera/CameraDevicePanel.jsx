import React, { useRef, useState } from 'react';
import { useSelector } from 'react-redux';
import { Card, CardHeader, IconButton, Box } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import dayjs from 'dayjs';

import CloseIcon from '@mui/icons-material/Close';
import ZoomOutMapIcon from '@mui/icons-material/ZoomOutMap';
import FullscreenExitIcon from '@mui/icons-material/FullscreenExit';
import MaximizeIcon from '@mui/icons-material/Maximize';
import MinimizeIcon from '@mui/icons-material/Minimize';
import CameraAltIcon from '@mui/icons-material/CameraAlt';

import CameraDevice, { customEqual } from './CameraDevice';

const useStyles = makeStyles()((theme) => ({
  card: {
    pointerEvents: 'auto',
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: 'difference',
  },
}));

const SIZE = { MIN: 'min', MED: 'med', MAX: 'max' };

const SIZE_CONFIG = {
  [SIZE.MIN]: {
    frame: { width: '20vw', aspectRatio: '16 / 9' },
    root: { left: '360px', top: '96px', transform: 'translateX(1%)' },
  },
  [SIZE.MED]: {
    frame: { width: '40vw', aspectRatio: '16 / 9' },
    root: { left: '360px', top: '96px', transform: 'translateX(1%)' },
  },
  [SIZE.MAX]: {
    frame: { width: '95vw', aspectRatio: '16 / 9' },
    root: { left: '51%', top: '5%', transform: 'translateX(-50%)' },
  },
};

// Floating PiP-style panel used over the map (MainPage/MainPage3D): owns the
// window chrome (drag-free position, min/max/close, capture button) and
// delegates the actual video/frame rendering to the CameraDevice selector.
const CameraDevicePanel = React.memo(({ deviceId, onClose }) => {
  const { classes } = useStyles();
  const device = useSelector((state) => state.devices.items[deviceId], customEqual);

  const [cardSize, setCardSize] = useState(SIZE.MIN);
  const cameraRef = useRef(null);

  const { frame: frameSx, root: rootSx } = SIZE_CONFIG[cardSize];

  const toggleCollapse = () => setCardSize((s) => (s === SIZE.MIN ? SIZE.MED : SIZE.MIN));
  const toggleFullscreen = () => setCardSize((s) => (s === SIZE.MAX ? SIZE.MED : SIZE.MAX));

  const closeCard = () => {
    onClose();
    setCardSize(SIZE.MED);
  };

  const handleCapture = () => {
    const dataUrl = cameraRef.current?.capture();
    if (!dataUrl) return;

    const timestamp = dayjs().format('YYYYMMDD_HHmmss');
    const fileName = `snap_${cameraRef.current?.cameraSrc || deviceId}_${timestamp}.jpg`;
    const link = document.createElement('a');
    link.href = dataUrl;
    link.download = fileName;
    link.click();
  };

  return (
    <Box
      sx={{
        pointerEvents: 'none',
        position: 'fixed',
        zIndex: 6,
        ...rootSx,
      }}
    >
      {device && (
        <Card elevation={3} className={classes.card}>
          <CardHeader
            title={device.name}
            slotProps={{ title: { variant: 'subtitle2' } }}
            sx={{ py: 1, px: 1 }}
            action={
              <Box sx={{ display: 'flex' }}>
                <IconButton size="small" onClick={handleCapture}>
                  <CameraAltIcon fontSize="small" className={classes.mediaButton} />
                </IconButton>
                <IconButton size="small" onClick={toggleCollapse}>
                  {cardSize === SIZE.MIN ? (
                    <MaximizeIcon fontSize="small" className={classes.mediaButton} />
                  ) : (
                    <MinimizeIcon fontSize="small" className={classes.mediaButton} />
                  )}
                </IconButton>
                <IconButton size="small" onClick={toggleFullscreen}>
                  {cardSize === SIZE.MAX ? (
                    <FullscreenExitIcon fontSize="small" className={classes.mediaButton} />
                  ) : (
                    <ZoomOutMapIcon fontSize="small" className={classes.mediaButton} />
                  )}
                </IconButton>
                <IconButton size="small" onClick={closeCard}>
                  <CloseIcon fontSize="small" className={classes.mediaButton} />
                </IconButton>
              </Box>
            }
          />
          <Box
            sx={{
              ...frameSx,
              display: 'flex',
              justifyContent: 'center',
              alignItems: 'center',
              background: 'black',
              '& > *': { width: '100%', height: '100%' },
            }}
          >
            <CameraDevice ref={cameraRef} deviceId={deviceId} />
          </Box>
        </Card>
      )}
    </Box>
  );
});

export default CameraDevicePanel;
