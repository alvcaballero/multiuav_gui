import React, { useRef, useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Card, IconButton, CardMedia, ButtonGroup } from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
import ZoomOutMapIcon from '@mui/icons-material/ZoomOutMap';
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
    //position: "fixed",
  },
  root_max: {
    pointerEvents: 'none',
    //position: "fixed",
  },
}));

export const CameraWebRTCV4 = ({
  deviceId,
  deviceIp = '127.0.0.1',
  camera_src = 'video0',
  onClose,
}) => {
  const { classes } = useStyles();
  //const camera_stream ="20"// useSelector((state) => state.session.camera[deviceId]);
  const device = deviceId
    ? useSelector((state) => state.devices.items[deviceId])
    : { name: 'test' };
  const deviceip = 'http://' + deviceIp + ':8889/' + camera_src; //device?.ip;
  const [maxsize, setmaxsize] = useState(false);
  let btn_class = classes.card;
  let rootclass = classes.root_max;
  let frameclass = classes.media;
  function Changemaxsize() {
    setmaxsize(!maxsize);
  }
  const restartPause = 2000;
  const localVideoRef = useRef();

  function closecard() {
    onClose();
    setmaxsize(false);
  }

  return (
    <div className={rootclass}>
      {device && (
        <Card className={btn_class}>
          <div>
            <div className={classes.tittle}>{'Id: ' + device.name}</div>
          </div>
          <iframe src={deviceip} className={frameclass} />
        </Card>
      )}
    </div>
  );
};
