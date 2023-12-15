import React, { useRef, useEffect, useState } from "react";
import novideo from "../assets/img/video_loading.mp4";
import { useDispatch, useSelector } from "react-redux";
import Draggable from "react-draggable";
import makeStyles from "@mui/styles/makeStyles";
import { Card, IconButton, CardMedia } from "@mui/material";
import CloseIcon from "@mui/icons-material/Close";
import ZoomOutMapIcon from "@mui/icons-material/ZoomOutMap";

const useStyles = makeStyles((theme) => ({
  card: {
    pointerEvents: "auto",
    width: theme.dimensions.popupMaxWidth,
  },
  card_max: {
    pointerEvents: "auto",
    width: "100%",
  },
  media: {
    //height: theme.dimensions.popupImageHeight,
    width: "100%",
    display: "flex",
    justifyContent: "flex-end",
    alignItems: "flex-start",
    background: "black",
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: "difference",
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
    width: "55%",
  },
}));

export const CameraWebRTCV2 = ({ deviceId, deviceIp, camera_src, onClose }) => {
  const classes = useStyles();
  const camera_stream = useSelector((state) => state.data.camera[deviceId]);
  const device = useSelector((state) => state.devices.items[deviceId]);
  const deviceip = "http://" + deviceIp + ":8889/" + camera_src + "/" + "whep"; //device?.ip;
  const [maxsize, setmaxsize] = useState(false);
  const [instCP, setinstCP] = useState(false);
  let btn_class = maxsize ? classes.card_max : classes.card;
  let rootclass = maxsize ? classes.root_max : classes.root;
  //var mypc = null;
  function Changemaxsize() {
    setmaxsize(!maxsize);
  }
  const restartPause = 2000;
  const localVideoRef = useRef();

  const linkToIceServers = (links) =>
    links !== null
      ? links.split(", ").map((link) => {
          const m = link.match(
            /^<(.+?)>; rel="ice-server"(; username="(.*?)"; credential="(.*?)"; credential-type="password")?/i
          );
          const ret = {
            urls: [m[1]],
          };

          if (m[3] !== undefined) {
            ret.username = m[3];
            ret.credential = m[4];
            ret.credentialType = "password";
          }

          return ret;
        })
      : [];

  const parseOffer = (offer) => {
    const ret = {
      iceUfrag: "",
      icePwd: "",
      medias: [],
    };

    for (const line of offer.split("\r\n")) {
      if (line.startsWith("m=")) {
        ret.medias.push(line.slice("m=".length));
      } else if (ret.iceUfrag === "" && line.startsWith("a=ice-ufrag:")) {
        ret.iceUfrag = line.slice("a=ice-ufrag:".length);
      } else if (ret.icePwd === "" && line.startsWith("a=ice-pwd:")) {
        ret.icePwd = line.slice("a=ice-pwd:".length);
      }
    }
    return ret;
  };

  const generateSdpFragment = (myofferData, candidates) => {
    const candidatesByMedia = {};
    for (const candidate of candidates) {
      const mid = candidate.sdpMLineIndex;
      if (candidatesByMedia[mid] === undefined) {
        candidatesByMedia[mid] = [];
      }
      candidatesByMedia[mid].push(candidate);
    }

    let frag =
      "a=ice-ufrag:" +
      myofferData.iceUfrag +
      "\r\n" +
      "a=ice-pwd:" +
      myofferData.icePwd +
      "\r\n";

    let mid = 0;

    for (const media of myofferData.medias) {
      if (candidatesByMedia[mid] !== undefined) {
        frag += "m=" + media + "\r\n" + "a=mid:" + mid + "\r\n";

        for (const candidate of candidatesByMedia[mid]) {
          frag += "a=" + candidate.candidate + "\r\n";
        }
      }
      mid++;
    }
    return frag;
  };

  class WHEPClient {
    constructor() {
      this.pc = null;
      this.restartTimeout = null;
      this.eTag = "";
      this.queuedCandidates = [];
      this.start();
    }

    start() {
      console.log("requesting ICE servers");

      fetch(deviceip, {
        method: "OPTIONS",
      })
        .then((res) => this.onIceServers(res))
        .catch((err) => {
          console.log("error: " + err);
          this.scheduleRestart();
        });
    }

    onIceServers(res) {
      this.pc = new RTCPeerConnection({
        iceServers: linkToIceServers(res.headers.get("Link")),
      });

      const direction = "sendrecv";
      this.pc.addTransceiver("video", { direction });
      this.pc.addTransceiver("audio", { direction });

      this.pc.onicecandidate = (evt) => this.onLocalCandidate(evt);
      this.pc.oniceconnectionstatechange = () => this.onConnectionState();

      this.pc.ontrack = (evt) => {
        console.log("new track:", evt.track.kind);
        localVideoRef.current.srcObject = evt.streams[0];
      };

      this.pc.createOffer().then((desc) => {
        this.offerData = parseOffer(desc.sdp);
        this.pc.setLocalDescription(desc);

        console.log("sending offer");

        fetch(deviceip, {
          method: "POST",
          headers: {
            "Content-Type": "application/sdp",
          },
          body: desc.sdp,
        })
          .then((res) => {
            if (res.status !== 201) {
              throw new Error("bad status code");
            }
            this.eTag = res.headers.get("E-Tag");
            return res.text();
          })
          .then((sdp) =>
            this.onRemoteDescription(
              new RTCSessionDescription({
                type: "answer",
                sdp,
              })
            )
          )
          .catch((err) => {
            console.log("error: " + err);
            this.scheduleRestart();
          });
      });
    }

    onConnectionState() {
      if (this.restartTimeout !== null) {
        return;
      }

      console.log("peer connection state:", this.pc.iceConnectionState);

      switch (this.pc.iceConnectionState) {
        case "disconnected":
          this.scheduleRestart();
      }
    }

    onRemoteDescription(answer) {
      if (this.restartTimeout !== null) {
        return;
      }

      this.pc.setRemoteDescription(new RTCSessionDescription(answer));

      if (this.queuedCandidates.length !== 0) {
        this.sendLocalCandidates(this.queuedCandidates);
        this.queuedCandidates = [];
      }
    }

    onLocalCandidate(evt) {
      if (this.restartTimeout !== null) {
        return;
      }

      if (evt.candidate !== null) {
        if (this.eTag === "") {
          this.queuedCandidates.push(evt.candidate);
        } else {
          this.sendLocalCandidates([evt.candidate]);
        }
      }
    }

    sendLocalCandidates(candidates) {
      fetch(deviceip, {
        method: "PATCH",
        headers: {
          "Content-Type": "application/trickle-ice-sdpfrag",
          "If-Match": this.eTag,
        },
        body: generateSdpFragment(this.offerData, candidates),
      })
        .then((res) => {
          if (res.status !== 204) {
            throw new Error("bad status code");
          }
        })
        .catch((err) => {
          console.log("error: " + err);
          this.scheduleRestart();
        });
    }

    scheduleRestart() {
      if (this.restartTimeout !== null) {
        return;
      }
      if (this.pc !== null) {
        this.pc.close();
        this.pc = null;
      }
      this.restartTimeout = window.setTimeout(() => {
        this.restartTimeout = null;
        this.start();
      }, restartPause);
      this.eTag = "";
      this.queuedCandidates = [];
    }

    close() {
      if (
        this.pc &&
        (this.pc.connectionState === "connected" ||
          this.pc.connectionState === "connecting")
      ) {
        // Close the RTCPeerConnection
        this.pc.close();
        console.log("WebRTC session closed.");
      } else {
        console.log("No active WebRTC session to close.");
      }
    }
  }

  function closecard() {
    onClose();
    setmaxsize(false);
    console.log("close card");
    console.log(instCP);
    if (instCP) {
      console.log("close card2");
      instCP.close();
      setinstCP(null);
    }
  }

  useEffect(() => {
    console.log(
      "device in camera-" + device + "-" + deviceIp + "+" + camera_src
    );
    if (deviceId) {
      //ocalVideoRef.current.srcObject = novideo;
      //localVideoRef.current.play();
      setinstCP(new WHEPClient());
      //instCP.start();
      console.log("crear");
    } // here initial your data with default value
    //console.log(instCP)
  }, [deviceId]);

  return (
    <div className={rootclass}>
      {device && (
        <Card elevation={3} className={btn_class}>
          <div
            style={{
              display: "flex",
              right: "5px",
              height: "35px",
              position: "absolute",
            }}
          >
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

          <div
            style={{
              display: "block",
              width: "calc( 100% - 70pt )",
              paddingLeft: "15pt",
              paddingTop: "10pt",
              paddingBottom: "10pt",
              textAlign: "left",
            }}
          >
            {" "}
            {"Image " + device.name}{" "}
          </div>

          <video
            ref={localVideoRef}
            autoPlay={true}
            playsInline={true}
            className={classes.media}
          >
            <source src={novideo} type="video/mp4" />
          </video>
        </Card>
      )}
    </div>
  );
};
