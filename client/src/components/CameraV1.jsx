import React, { useRef, useEffect, useState } from 'react';
import novideo from '../resources/images/placeholder.jpg';
import { useDispatch, useSelector } from 'react-redux';
import { Card, IconButton, CardMedia } from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
import ZoomOutMapIcon from '@mui/icons-material/ZoomOutMap';
import { makeStyles } from 'tss-react/mui';


const useStyles = makeStyles()((theme) => ({
  card: {
    pointerEvents: 'auto',
  },
  media: {
    //height: theme.dimensions.popupImageHeight,
    width: '100%',
    height: '20vw',
    //display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
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
  },
  root_max: {
    pointerEvents: 'none',
  },
}));

export const CameraV1 = ({ deviceId, datacamera, onClose }) => {
  const { classes } = useStyles();
  const [camera_image, setcamera_image] = useState(novideo);

  const device = useSelector((state) => state.devices.items[deviceId]);

  const [maxsize, setmaxsize] = useState(false);
  let btn_class = classes.card;
  let rootclass = classes.root_max;
  const cameradata = useSelector((state) => state.session.camera[deviceId]);

  function Changemaxsize() {
    setmaxsize(!maxsize);
  }
  useEffect(() => {
    if (deviceId != null) {
      if (cameradata != null) {
        setcamera_image('data:image/bgr8;base64,' + cameradata.camera);
      } else {
        setcamera_image(novideo);
      }
    }
  }, [cameradata]);

  return (
    <div className={rootclass}>
      {device && (
        <Card className={btn_class}>
          <div>
            <div className={classes.tittle}>{'Image ' + device.name} </div>
          </div>

          <img src={camera_image} className={classes.media} />
        </Card>
      )}
    </div>
  );
};
