import React, { useEffect, useState } from 'react';
import { useSelector } from 'react-redux';
import { Card, IconButton } from '@mui/material';
import { makeStyles } from 'tss-react/mui';

import CloseIcon from '@mui/icons-material/Close';
import ZoomOutMapIcon from '@mui/icons-material/ZoomOutMap';
import FullscreenExitIcon from '@mui/icons-material/FullscreenExit';
import MaximizeIcon from '@mui/icons-material/Maximize';
import MinimizeIcon from '@mui/icons-material/Minimize';

import novideo from '../resources/images/placeholder.jpg';

const useStyles = makeStyles()((theme) => ({
  card: {
    pointerEvents: 'auto',
  },
  media: {
    width: theme.dimensions.popupMaxWidth,
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
    background: 'black',
  },
  mediaMed: {
    width: '45vw',
    height: '40vh',
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
    background: 'black',
  },
  mediaMax: {
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

const size = { min: 0, med: 2, max: 1 };

const RenderImages = ({ datacamera }) => {
  const [camera_image, setcamera_image] = useState(novideo);

  useEffect(() => {
    if (datacamera != null) {
      setcamera_image('data:image/bgr8;base64,' + datacamera.camera);
    } else {
      setcamera_image(novideo);
    }
  }, [datacamera]);
  return <img src={camera_image} style={{ width: '100%' }} />;
};

const customEqual = (oldValue, newValue) => {
  return oldValue?.camera === newValue?.camera && oldValue?.ip === newValue?.ip && oldValue?.name === newValue?.name;
};

/**
 * CameraDevice component displays a video stream from a specified device using WebRTC WebRTC_env or Websocket.
 *
 * @param {Object} props - The properties object.
 * @param {string} props.device - The device for get de streaming of video.
 * @param {function} props.onClose - The function to call when the close button is clicked.
 * @param {string} [props.datacamera] - for  Websocket image '.
 *
 * @returns {JSX.Element} The CameraWebRTCV3 component.
 */

const CameraDevice = React.memo(({ deviceId, onClose }) => {
  const { classes } = useStyles();
  const device = useSelector((state) => state.devices.items[deviceId], customEqual);
  const datacamera = useSelector((state) => state.session.camera[deviceId]);

  const myhostname = `${window.location.hostname}`;

  const [cardSize, setCardSize] = useState(size.min);
  const [type, setType] = useState('Websocket');
  const [cameraSrc, setCameraSrc] = useState('');
  const [srcIp, setSrcIp] = useState('');

  const rootClass = cardSize === size.min ? classes.root : cardSize === size.med ? classes.root : classes.root_max;
  const frameClass =
    cardSize === size.min ? classes.media : cardSize === size.med ? classes.mediaMed : classes.mediaMax;

  const ChangeMaxSize = () => {
    setCardSize(cardSize === size.max ? size.min : size.max);
  };
  const ChangeMedSize = () => {
    setCardSize(cardSize === size.med ? size.min : size.med);
  };

  const closeCard = () => {
    onClose();
    setCardSize(size.min);
  };

  useEffect(() => {
    if (device) {
      console.log(`device in camera ${device.name}`);
      if (device.camera.length > 0) {
        console.log(`device in camera ${device.camera[0].type} - ${device.camera[0].source}`);
        setType(device.camera[0].type);
        if (device.camera[0].type === 'WebRTC') {
          setCameraSrc(`${device.name}_${device.camera[0].source}`);
          setSrcIp(myhostname);
        } else {
          setCameraSrc(device.camera[0].source);
          setSrcIp(device.ip);
        }
      }
    }
  }, [deviceId]);

  return (
    <div className={rootClass}>
      {device && (
        <Card elevation={3} className={classes.card}>
          <div className={classes.gruopBtn}>
            <IconButton size="small" onClick={ChangeMedSize}>
              {cardSize === size.med ? (
                <MinimizeIcon fontSize="small" className={classes.mediaButton} />
              ) : (
                <MaximizeIcon fontSize="small" className={classes.mediaButton} />
              )}
            </IconButton>
            <IconButton size="small" onClick={ChangeMaxSize}>
              {cardSize === size.max ? (
                <FullscreenExitIcon fontSize="small" className={classes.mediaButton} />
              ) : (
                <ZoomOutMapIcon fontSize="small" className={classes.mediaButton} />
              )}
            </IconButton>
            <IconButton size="small" onClick={closeCard}>
              <CloseIcon fontSize="small" className={classes.mediaButton} />
            </IconButton>
          </div>
          <div className={classes.tittle}>{` ${device.name}`}</div>
          {type === 'Websocket' ? (
            <div className={frameClass}>
              <RenderImages datacamera={datacamera} Myclass={frameClass} />
            </div>
          ) : (
            <iframe src={`http://${srcIp}:8889/${cameraSrc}`} className={frameClass} title={`Image ${device.name}`} />
          )}
        </Card>
      )}
    </div>
  );
});

export default CameraDevice;
