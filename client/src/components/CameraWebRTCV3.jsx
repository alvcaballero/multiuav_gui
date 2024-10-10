import React, { useRef, useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Card, IconButton, CardMedia, ButtonGroup } from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
import ZoomOutMapIcon from '@mui/icons-material/ZoomOutMap';
import makeStyles from '@mui/styles/makeStyles';

const useStyles = makeStyles((theme) => ({
  card: {
    pointerEvents: 'auto',
  },
  media: {
    //height: theme.dimensions.popupImageHeight,
    width: theme.dimensions.popupMaxWidth,
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
    background: 'black',
  },
  media1: {
    //height: theme.dimensions.popupImageHeight
    width: '95vw',
    height: '90vh',
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
    background: 'black',
  },
  gruopBtn: {
    display: 'flex',
    right: '5px',
    height: '40px',
    position: 'absolute',
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: 'difference',
  },
  tittle: {
    display: 'block',
    width: 'calc( 100% - 60pt )',
    paddingLeft: '15pt',
    paddingTop: '10pt',
    paddingBottom: '10pt',
    textAlign: 'left',
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: theme.spacing(1, 1, 0, 2),
  },
  root: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 6,
    left: '360px',
    top: theme.spacing(15),
    transform: 'translateX(1%)',
  },
  root_max: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 6,
    left: '51%',
    top: '5%',
    transform: 'translateX(-50%)',
  },
}));

const CameraWebRTCV3 = ({ deviceId, deviceIp, camera_src, onClose }) => {
  const classes = useStyles();
  // const camera_stream = useSelector((state) => state.data.camera[deviceId]);
  const device = useSelector((state) => state.devices.items[deviceId]);
  const videoUrl = `http://${deviceIp}:8889/${camera_src}`; //device?.ip;
  const [maxsize, setMaxSize] = useState(false);

  const btnClass = classes.card;
  const rootClass = maxsize ? classes.root_max : classes.root;
  const frameClass = maxsize ? classes.media1 : classes.media;

  const ChangeMaxSize = () => {
    setMaxSize(!maxsize);
  };

  const closeCard = () => {
    onClose();
    setMaxSize(false);
  };

  useEffect(() => {
    if (deviceId) {
      console.log(`device in camera ${device.name} - ${deviceIp}  - ${camera_src}`);
    }
  }, [deviceId]);

  return (
    <div className={rootClass}>
      {device && (
        <Card elevation={3} className={btnClass}>
          <div className={classes.gruopBtn}>
            <IconButton size="small" onClick={ChangeMaxSize} onTouchStart={ChangeMaxSize}>
              <ZoomOutMapIcon fontSize="small" className={classes.mediaButton} />
            </IconButton>
            <IconButton size="small" onClick={closeCard}>
              <CloseIcon fontSize="small" className={classes.mediaButton} />
            </IconButton>
          </div>
          <div className={classes.tittle}>{`Image${device.name}`}</div>
          <iframe src={videoUrl} className={frameClass} title={`Image${device.name}`} />
        </Card>
      )}
    </div>
  );
};

export default CameraWebRTCV3;
