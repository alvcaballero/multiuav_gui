import React, { useContext, useState, Fragment, useRef } from "react";
import RoutesList from "./RoutesList";
import { Divider, Typography, IconButton, Toolbar } from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";
import { useNavigate } from "react-router-dom";
import { RosControl, RosContext } from "./RosControl";
import ArrowBackIcon from "@mui/icons-material/ArrowBack";
import UploadFileIcon from "@mui/icons-material/UploadFile";
import SaveAltIcon from "@mui/icons-material/SaveAlt";
import DeleteIcon from "@mui/icons-material/Delete";

const useStyles = makeStyles((theme) => ({
  toolbar: {
    display: "flex",
    gap: "10px 10px",
    height: "30px",
    borderBottom: "3px solid rgb(212, 212, 212)",
  },
  list: {
    maxHeight: `calc(100vh - 152px)`,
    overflowY: "auto",
  },
  title: {
    flexGrow: 1,
  },
  fileInput: {
    display: "none",
  },
}));

export const MissionPanel = () => {
  const classes = useStyles();
  const navigate = useNavigate();
  const rosContex = useContext(RosContext);
  const scroolRef = useRef(null);

  const [mission, setmission] = useState({
    name: "no mission",
    description: "",
    route: [],
  });

  const setScrool = (value) => {
    console.log("scrool" + value);
    setTimeout(() => {
      scroolRef.current.scroll(0, value);
    }, 1000);
  };

  const readFile = (e) => {
    //https://www.youtube.com/watch?v=K3SshoCXC2g
    const file = e.target.files[0];
    if (!file) return;

    const fileReader = new FileReader();
    fileReader.readAsText(file);
    fileReader.onload = () => {
      console.log(fileReader.result);
      console.log(file.name);
      rosContex.openMision(file.name, fileReader.result);
    };
    fileReader.onerror = () => {
      console.log(fileReader.error);
    };
  };

  const DeleteMission = () => {
    setmission({ name: "no mission", description: "", route: [] });
  };
  const SaveMission = () => {
    const fileData = YAML.stringify(mission);
    //console.log(YAML.stringify(mission,))
    const blob = new Blob([fileData], { type: "text/plain" });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.download = "mission-" + mission.name + ".yaml";
    link.href = url;
    link.click();
  };
  return (
    <Fragment>
      <Toolbar className={classes.toolbar}>
        <IconButton edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
          <ArrowBackIcon />
        </IconButton>
        <Typography variant="h6" className={classes.title}>
          Mission Task
        </Typography>
        <IconButton onClick={SaveMission}>
          <SaveAltIcon />
        </IconButton>
        <IconButton onClick={DeleteMission}>
          <DeleteIcon />
        </IconButton>
        <label htmlFor="upload-gpx">
          <input
            accept=".yaml, .plan, .waypoint, .kml"
            id="upload-gpx"
            type="file"
            className={classes.fileInput}
            onChange={readFile}
          />
          <IconButton edge="end" component="span" onClick={() => {}}>
            <UploadFileIcon />
          </IconButton>
        </label>
      </Toolbar>
      <div ref={scroolRef} className={classes.list}>
        <RoutesList
          mission={mission}
          setmission={setmission}
          setScrool={setScrool}
        />
      </div>
    </Fragment>
  );
};
