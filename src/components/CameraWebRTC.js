import React,{ useRef,useEffect,useState} from 'react'
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

export const CameraWebRTC = ({ deviceId,position, onClose}) => {
  const classes = useClasses(styles);
  const [camera_image, setcamera_image] = useState(novideo);
  const [img, setimg] = useState(novideo);
  const camera_stream = useSelector((state) => state.data.camera[deviceId]);
  const device = useSelector((state) => state.devices.items[deviceId]);
  const mediaStream = new MediaStream();

  const restartPause = 2000;
  const localVideoRef = useRef();

  let ws = useRef(null);
  let pc = useRef(null);
  let restartTimeout = useRef(null);
  let terminated = false;


  useEffect(()=>{
		start();
    // here initial your data with default value
  
  }, [deviceId])



  function start() {
		console.log("connecting");

        ws = new WebSocket('ws://localhost:8889/test/' + 'ws');

        ws.onerror = () => {
            console.log("ws error");
            if (ws === null) {
                return;
            }
            ws.close();
            ws = null;
        };

        ws.onclose = () => {
            console.log("ws closed");
            ws = null;
            scheduleRestart();
        };

        ws.onmessage = (msg) => onIceServers(msg);
	}

  function onIceServers(msg) {
    if (ws === null) {
        return;
    }

    const iceServers = JSON.parse(msg.data);

    pc = new RTCPeerConnection({
        iceServers,
    });

    ws.onmessage = (msg) => onRemoteDescription(msg);
    pc.onicecandidate = (evt) => onIceCandidate(evt);

    pc.oniceconnectionstatechange = () => {
        if (pc === null) {
            return;
        }

        console.log("peer connection state:", pc.iceConnectionState);

        switch (pc.iceConnectionState) {
        case "disconnected":
            scheduleRestart();
        }
    };

    pc.ontrack = (evt) => {
        console.log("new track " + evt.track.kind);
        localVideoRef.current.srcObject = evt.streams[0];
    };

    const direction = "sendrecv";
    pc.addTransceiver("video", { direction });
    pc.addTransceiver("audio", { direction });

    pc.createOffer()
        .then((desc) => {
            if (pc === null || ws === null) {
                return;
            }

            pc.setLocalDescription(desc);

            console.log("sending offer");
            ws.send(JSON.stringify(desc));
        });
  } 
  function onRemoteDescription(msg) {
		if (pc === null || ws === null) {
			return;
		}

		pc.setRemoteDescription(new RTCSessionDescription(JSON.parse(msg.data)));
		ws.onmessage = (msg) => onRemoteCandidate(msg);
	}

  function onIceCandidate(evt) {
        if (ws === null) {
            return;
        }

        if (evt.candidate !== null) {
            if (evt.candidate.candidate !== "") {
                ws.send(JSON.stringify(evt.candidate));
            }
        }
    }

	function onRemoteCandidate(msg) {
		if (pc === null) {
			return;
		}

		pc.addIceCandidate(JSON.parse(msg.data));
	}

  function scheduleRestart() {
        if (terminated) {
            return;
        }

        if (ws !== null) {
            ws.close();
            ws = null;
        }

        if (pc !== null) {
            pc.close();
            pc = null;
        }

        restartTimeout = window.setTimeout(() => {
            restartTimeout = null;
            start();
        }, restartPause);
    }

  return (
    <div className={classes.root}>
      {device && (
      <Card elevation={3} className={classes.card}>
        {false && <img src={camera_image} className={classes.media} />}
        <video  ref={localVideoRef} autoPlay playsInline></video>

          <IconButton
            size="small"
            onClick={onClose}
            onTouchStart={onClose}
          >
            <CloseIcon fontSize="small" className={classes.mediaButton} />
          </IconButton>
      </Card>)}
  </div>
  )
}
