import React, { useEffect, useMemo, useState, useRef } from 'react';
import { useSelector } from 'react-redux';
import { Card, CardHeader, IconButton, CircularProgress, Typography, Box } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import dayjs from 'dayjs';

import CloseIcon from '@mui/icons-material/Close';
import ZoomOutMapIcon from '@mui/icons-material/ZoomOutMap';
import FullscreenExitIcon from '@mui/icons-material/FullscreenExit';
import MaximizeIcon from '@mui/icons-material/Maximize';
import MinimizeIcon from '@mui/icons-material/Minimize';
import CameraAltIcon from '@mui/icons-material/CameraAlt';

import novideo from '../../resources/images/placeholder.jpg';
import { MediaMTXWebRTCReader } from './MediaMTXWebRTCReader';

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
    frame: { width: '320px', height: '16vh' },
    root: { left: '360px', top: '96px', transform: 'translateX(1%)' },
  },
  [SIZE.MED]: {
    frame: { width: '40vw', height: '59vh' },
    root: { left: '360px', top: '96px', transform: 'translateX(1%)' },
  },
  [SIZE.MAX]: {
    frame: { width: '95vw', height: '90vh' },
    root: { left: '51%', top: '5%', transform: 'translateX(-50%)' },
  },
};

const hostname = window.location.hostname;

const RenderImages = ({ datacamera }) => {
  const [camera_image, setCameraImage] = useState(novideo);

  useEffect(() => {
    if (datacamera != null) {
      setCameraImage('data:image/jpeg;base64,' + datacamera.camera);
    } else {
      setCameraImage(novideo);
    }
  }, [datacamera]);
  return <img src={camera_image} style={{ width: '100%' }} />;
};

const MediaMTXPlayer = ({ src, videoRef }) => {
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (!src) return;
    setLoading(true);
    setError(null);

    const whepUrl = src.endsWith('/') ? `${src}whep` : `${src}/whep`;
    const videoEl = videoRef.current;

    const reader = new MediaMTXWebRTCReader({
      url: whepUrl,
      onTrack: (evt) => {
        if (videoRef.current) {
          videoRef.current.srcObject = evt.streams[0];
        }
      },
      onError: (err) => {
        setLoading(false);
        setError(err);
        console.error('MediaMTX Reader Error:', err);
      },
    });

    return () => {
      reader.close();
      if (videoEl) videoEl.srcObject = null;
    };
  }, [src, videoRef]);

  return (
    <Box
      sx={{
        position: 'relative',
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        background: 'black',
        overflow: 'hidden',
        width: '100%',
        height: '100%',
      }}
    >
      {loading && (
        <Box
          sx={{
            position: 'absolute',
            top: '50%',
            left: '50%',
            transform: 'translate(-50%, -50%)',
          }}
        >
          <CircularProgress aria-label="Loading…" />
        </Box>
      )}
      {error && !loading && (
        <Box
          sx={{
            position: 'absolute',
            top: '50%',
            left: '50%',
            transform: 'translate(-50%, -50%)',
            textAlign: 'center',
            color: '#888',
          }}
        >
          <Typography variant="caption" display="block">
            STREAM NOT AVAILABLE
          </Typography>
        </Box>
      )}
      <video
        ref={videoRef}
        autoPlay
        muted
        playsInline
        onCanPlay={() => setLoading(false)}
        style={{
          objectFit: 'contain',
          width: '100%',
          height: '100%',
          display: loading || error ? 'none' : 'block',
        }}
      />
    </Box>
  );
};

const customEqual = (oldValue, newValue) => {
  return (
    oldValue?.camera === newValue?.camera &&
    oldValue?.ip === newValue?.ip &&
    oldValue?.name === newValue?.name
  );
};

const CameraDevice = React.memo(({ deviceId, onClose }) => {
  const { classes } = useStyles();
  const device = useSelector((state) => state.devices.items[deviceId], customEqual);
  const datacamera = useSelector((state) => state.session.camera[deviceId]);

  const [cardSize, setCardSize] = useState(SIZE.MED);
  const videoRef = useRef(null);

  const { type, cameraSrc, srcIp } = useMemo(() => {
    const cam = device?.camera?.[0];
    if (!cam) return { type: 'Websocket', cameraSrc: '', srcIp: '' };
    if (cam.type === 'WebRTC') {
      return { type: 'WebRTC', cameraSrc: `${device.name}_${cam.source}`, srcIp: hostname };
    }
    return { type: cam.type, cameraSrc: cam.source, srcIp: device.ip };
  }, [device]);

  const { frame: frameSx, root: rootSx } = SIZE_CONFIG[cardSize];

  const toggleCollapse = () => setCardSize((s) => (s === SIZE.MIN ? SIZE.MED : SIZE.MIN));
  const toggleFullscreen = () => setCardSize((s) => (s === SIZE.MAX ? SIZE.MED : SIZE.MAX));

  const closeCard = () => {
    onClose();
    setCardSize(SIZE.MED);
  };

  const handleCapture = () => {
    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');
    const timestamp = dayjs().format('YYYYMMDD_HHmmss');
    const fileName = `snap_${cameraSrc}_${timestamp}.jpg`;

    if (type === 'Websocket' && datacamera) {
      const link = document.createElement('a');
      link.href = 'data:image/jpeg;base64,' + datacamera.camera;
      link.download = fileName;
      link.click();
    } else if (videoRef.current) {
      canvas.width = videoRef.current.videoWidth;
      canvas.height = videoRef.current.videoHeight;
      ctx.drawImage(videoRef.current, 0, 0);
      const link = document.createElement('a');
      link.href = canvas.toDataURL('image/jpeg');
      link.download = fileName;
      link.click();
    }
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
              justifyContent: 'flex-end',
              alignItems: 'flex-start',
              background: 'black',
            }}
          >
            {type === 'Websocket' ? (
              <RenderImages datacamera={datacamera} />
            ) : (
              <MediaMTXPlayer src={`http://${srcIp}:8889/${cameraSrc}`} videoRef={videoRef} />
            )}
          </Box>
        </Card>
      )}
    </Box>
  );
});

export default CameraDevice;
