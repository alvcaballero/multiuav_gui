import React,{useRef,useEffect,useState} from 'react'
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
import ZoomOutMapIcon from '@mui/icons-material/ZoomOutMap';


const styles = theme => ({
  card: {
    pointerEvents: 'auto',
    width: theme.dimensions.popupMaxWidth,
  },
  card_max: {
    pointerEvents: 'auto',
    width: "100%",
  },
  media: {
    //height: theme.dimensions.popupImageHeight,
    width: "100%",
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
    zIndex: 6,
    left: '39%',
    top: theme.spacing(15),
    transform: 'translateX(-50%)',
  },
  root_max: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 6,
    left: '51%',
    top: '5%',
    transform: 'translateX(-50%)',
    width: '75%'
  },
});

export const Camera = ({ deviceId,datacamera, onClose}) => {
  const classes = useClasses(styles);
  const [camera_image, setcamera_image] = useState(novideo);
  const [maxsize, setmaxsize] = useState(false);
  const device = useSelector((state) => state.devices.items[deviceId]);
  
  let btn_class = maxsize ? classes.card_max: classes.card;
  let rootclass = maxsize ? classes.root_max: classes.root;

  function Changemaxsize(){
    setmaxsize(!maxsize);
  }
  useEffect(() => {
    if(deviceId != null){
      if(datacamera != null){
      setcamera_image("data:image/bgr8;base64,"+datacamera.camera)
      }else{
        setcamera_image(novideo)
      }
    }

  }, [datacamera]);


  return (
    <div className={rootclass}>
      {device && (
      <Card elevation={3} className={btn_class}>
          <div style={{display:'flex',right:"5px",height:"35px",position:"absolute"}}>
            <IconButton size="small" onClick={Changemaxsize} onTouchStart={Changemaxsize}>
              <ZoomOutMapIcon fontSize="small" className={classes.mediaButton} />
            </IconButton>
            <IconButton size="small" onClick={()=>{onClose();setmaxsize(false)}} onTouchStart={onClose}>
              <CloseIcon fontSize="small" className={classes.mediaButton} />
            </IconButton>
          </div>
        
          <div style={{display: 'block',width:"calc( 100% - 60pt )",paddingLeft:"15pt",paddingTop:"10pt",paddingBottom:"10pt",textAlign:"left"}}> {"Image "+device.name} </div>


        <img src={camera_image} className={classes.media} />


      </Card>)}
  </div>
  )
}
