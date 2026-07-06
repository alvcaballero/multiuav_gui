import React from 'react';
import novideo from '../../resources/images/placeholder.jpg';
import { useSelector } from 'react-redux';
import { Card } from '@mui/material';
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

export const CameraV1 = ({ deviceId }) => {
  const { classes } = useStyles();

  const device = useSelector((state) => state.devices.items[deviceId]);

  let btn_class = classes.card;
  let rootclass = classes.root_max;
  const cameradata = useSelector((state) => state.session.camera[deviceId]);

  const cameraImage =
    deviceId != null && cameradata != null
      ? 'data:image/bgr8;base64,' + cameradata.camera
      : novideo;

  return (
    <div className={rootclass}>
      {device && (
        <Card className={btn_class}>
          <div>
            <div className={classes.tittle}>{'Image ' + device.name} </div>
          </div>

          <img src={cameraImage} className={classes.media} alt={`${device.name} camera feed`} />
        </Card>
      )}
    </div>
  );
};
