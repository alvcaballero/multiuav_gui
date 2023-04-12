import React,{useEffect,useState} from 'react'
import novideo from '../assets/img/placeholder.jpg';
import { useDispatch, useSelector } from 'react-redux';
import Draggable from 'react-draggable';
import useClasses from './useClasses'
import {
  Card,
  IconButton,
  CardMedia,
} from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
const cameraStyle={
  height: '300px',
  width: '300px',
  objectFit: 'contain'
};

const styles = theme => ({
  card: {
    pointerEvents: 'auto',
    width: theme.dimensions.popupMaxWidth,
  },
  media: {
    height: theme.dimensions.popupImageHeight,
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: 'difference',
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
    zIndex: 5,
    left: '39%',
    [theme.breakpoints.up('md')]: {
      left: `calc(25% + 10/ 2)`,
      top: theme.spacing(15),
    },
    [theme.breakpoints.down('md')]: {
      left: '50%',
      top: `calc(${theme.spacing(15)} + 10 px)`,
    },
    transform: 'translateX(-50%)',
  },
});

export const Camera = ({ deviceId,position, onClose}) => {
  const classes = useClasses(styles);
  const [camera_image, setcamera_image] = useState('no load mission');
  const camera_stream = useSelector((state) => state.data.camera[deviceId]);
  const device = useSelector((state) => state.devices.items[deviceId]);
  useEffect(() => {
    setcamera_image(camera_stream);
  }, [camera_stream]);

  return (
    <div className={classes.root}>
      {device && (
      <Card elevation={3} className={classes.card}>
        <CardMedia
          className={classes.media}
          //style={{height: 0, paddingTop: '56.25%'}}
          image={camera_image}
        >
          <IconButton
            size="small"
            onClick={onClose}
            onTouchStart={onClose}
          >
            <CloseIcon fontSize="small" className={classes.mediaButton} />
          </IconButton>
        </CardMedia>
      </Card>)}
  </div>
  )
}
