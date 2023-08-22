import React, { useRef, useEffect, useState } from "react";
import novideo from "../assets/img/video_loading.mp4";
import { useDispatch, useSelector } from "react-redux";
import Draggable from "react-draggable";
import useClasses from "./useClasses";
import { Card, IconButton, CardMedia, ButtonGroup } from "@mui/material";
import CloseIcon from "@mui/icons-material/Close";
import ZoomOutMapIcon from "@mui/icons-material/ZoomOutMap";
import makeStyles from "@mui/styles/makeStyles";

const useStyles = makeStyles((theme) => ({
  card: {
    pointerEvents: "auto",
  },
  media: {
    //height: theme.dimensions.popupImageHeight,
    width: theme.dimensions.popupMaxWidth,
    display: "flex",
    justifyContent: "flex-end",
    alignItems: "flex-start",
    background: "black",
  },
  media1: {
    //height: theme.dimensions.popupImageHeight
    width: "95vw",
    height: "90vh",
    display: "flex",
    justifyContent: "flex-end",
    alignItems: "flex-start",
    background: "black",
  },
  gruopBtn: {
    display: "flex",
    right: "5px",
    height: "40px",
    position: "absolute",
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: "difference",
  },
  tittle: {
    display: "block",
    width: "calc( 100% - 60pt )",
    paddingLeft: "15pt",
    paddingTop: "10pt",
    paddingBottom: "10pt",
    textAlign: "left",
  },
  header: {
    display: "flex",
    justifyContent: "space-between",
    alignItems: "center",
    padding: theme.spacing(1, 1, 0, 2),
  },
  root: {
    pointerEvents: "none",
    position: "fixed",
    zIndex: 6,
    left: "360px",
    top: theme.spacing(15),
    transform: "translateX(1%)",
  },
  root_max: {
    pointerEvents: "none",
    position: "fixed",
    zIndex: 6,
    left: "51%",
    top: "5%",
    transform: "translateX(-50%)",
  },
}));

export const CameraWebRTCV3 = ({ deviceId, deviceIp, camera_src, onClose }) => {
  const classes = useStyles();
  const camera_stream = useSelector((state) => state.data.camera[deviceId]);
  const device = useSelector((state) => state.devices.items[deviceId]);
  const deviceip = "http://" + deviceIp + ":8889/" + camera_src; //device?.ip;
  const [maxsize, setmaxsize] = useState(false);
  let btn_class = classes.card;
  let rootclass = maxsize ? classes.root_max : classes.root;
  let frameclass = maxsize ? classes.media1 : classes.media;
  function Changemaxsize() {
    setmaxsize(!maxsize);
  }
  const restartPause = 2000;
  const localVideoRef = useRef();

  function closecard() {
    onClose();
    setmaxsize(false);
  }

  useEffect(() => {
    console.log(
      "device in camera-" + device + "-" + deviceIp + "+" + camera_src
    );
    if (deviceId) {
      console.log("crear");
    }
  }, [deviceId]);

  return (
    <div className={rootclass}>
      {device && (
        <Card elevation={3} className={btn_class}>
          <div className={classes.gruopBtn}>
            <IconButton
              size="small"
              onClick={Changemaxsize}
              onTouchStart={Changemaxsize}
            >
              <ZoomOutMapIcon
                fontSize="small"
                className={classes.mediaButton}
              />
            </IconButton>
            <IconButton size="small" onClick={closecard}>
              <CloseIcon fontSize="small" className={classes.mediaButton} />
            </IconButton>
          </div>
          <div className={classes.tittle}>{"Image " + device.name}</div>
          <iframe src={deviceip} className={frameclass} />
        </Card>
      )}
    </div>
  );
};
