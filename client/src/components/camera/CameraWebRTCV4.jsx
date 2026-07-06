import React from 'react';
import { useSelector } from 'react-redux';
import { Card } from '@mui/material';
import { makeStyles } from 'tss-react/mui';

const useStyles = makeStyles()((theme) => ({
  card: {
    pointerEvents: 'auto',
  },
  media: {
    //height: theme.dimensions.popupImageHeight,
    height: '20vw',
    width: '100%',
    //display: 'flex',
    //justifyContent: 'flex-end',
    //alignItems: 'flex-start',
    background: 'black',
  },
  media1: {
    //height: theme.dimensions.popupImageHeight
    width: '100%',
    height: '100%',
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
    background: 'black',
  },
  gruopBtn: {
    //display: "flex",
    //right: "5px",
    float: 'right',
    height: '40px',
    //position: "absolute",
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
  root: {
    pointerEvents: 'none',
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
  },
  root_max: {
    pointerEvents: 'none',
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
  },
}));

export const CameraWebRTCV4 = ({ deviceId, deviceIp = '127.0.0.1', camera_src = 'video0' }) => {
  const { classes } = useStyles();
  //const camera_stream ="20"// useSelector((state) => state.session.camera[deviceId]);
  const device = deviceId
    ? useSelector((state) => state.devices.items[deviceId])
    : { name: 'test' };
  const deviceip = 'http://' + deviceIp + ':8889/' + camera_src; //device?.ip;
  let btn_class = classes.card;
  let rootclass = classes.root_max;

  return (
    <div className={rootclass} style={{ flex: 1 }}>
      {device && (
        <Card
          className={btn_class}
          style={{ height: '100%', display: 'flex', flexDirection: 'column' }}
        >
          <div>
            <div className={classes.tittle}>{'Id: ' + device.name}</div>
          </div>
          <iframe src={deviceip} className={classes.media1} style={{ flex: 1, border: 'none' }} />
        </Card>
      )}
    </div>
  );
};
