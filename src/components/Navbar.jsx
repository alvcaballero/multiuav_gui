import React, { useState, useContext } from "react";
import { useDispatch } from "react-redux";
import { useNavigate } from "react-router-dom";
import "./Navbar.css";
import { map } from "../Mapview/Mapview";
import { RosContext } from "./RosControl";
import { AppBar, Toolbar, Container, Typography, Button } from "@mui/material";

import { missionActions } from "../store";

export const Navbar = ({ SetAddUAVOpen }) => {
  const [isActive, setIsActive] = useState(false);
  const dispatch = useDispatch();
  const navigate = useNavigate();

  const clearmission = () => {
    dispatch(missionActions.clearMission({}));
  };

  const handleClick = () => {
    // 👇️ toggle
    const element = document.getElementsByName("otrotest");
    //console.log(element);
    for (let i = 0; i < element.length; i++) {
      element[i].setAttribute("class", "mytest");
    }

    setTimeout(() => {
      console.log("1 Segundo esperado");
      for (let i = 0; i < element.length; i++) {
        element[i].setAttribute("class", "dropdown-content");
      }
    }, 500);
    setIsActive((current) => !current);

    // 👇️ or set to true
    // setIsActive(true);
  };
  const rosContex = useContext(RosContext);

  const readFile = (e) => {
    //https://www.youtube.com/watch?v=K3SshoCXC2g
    const file = e.target.files[0];
    if (!file) return;

    const fileReader = new FileReader();
    fileReader.readAsText(file);
    fileReader.onload = () => {
      //console.log(fileReader.result);
      //console.log(file.name);
      rosContex.openMision(file.name, fileReader.result);
    };
    fileReader.onerror = () => {
      console.log("error");
      console.log(fileReader.error);
    };
  };

  function sethome() {
    map.easeTo({
      center: [-6.0025, 37.412],
      zoom: Math.max(map.getZoom(), 5),
      offset: [0, -1 / 2],
    });
  }

  function openAddUav() {
    SetAddUAVOpen(true);
  }

  return (
    <AppBar
      position="static"
      style={{ backgroundColor: "#333", height: "52px" }}
    >
      <Container maxWidth="x">
        <Toolbar disableGutters>
          <Button
            onClick={() => {
              sethome();
              navigate("/");
            }}
          >
            <Typography
              variant="h6"
              noWrap
              component="a"
              sx={{
                mr: 2,
                display: { xs: "none", md: "flex" },
                fontFamily: "monospace",
                fontWeight: 700,
                letterSpacing: ".3rem",
                color: "#FFFFFF",
                textDecoration: "none",
              }}
            >
              Management Tool
            </Typography>
          </Button>
          <div className="dropdown">
            <button className="dropbtn">ROS </button>
            <div name="otrotest" className="dropdown-content">
              <a
                id="rosConnectNavbar"
                onClick={() => {
                  rosContex.rosConnect();
                  handleClick();
                }}
              >
                Connect Rosbridge Server
              </a>
            </div>
          </div>
          <div className="dropdown">
            <button className="dropbtn">
              UAV
              <i className="fa fa-caret-down"></i>
            </button>
            <div name="otrotest" className="dropdown-content">
              <a
                id="openAddUavNavbar"
                onClick={() => {
                  openAddUav();
                  handleClick();
                }}
              >
                Connect UAV
              </a>
              <a id="hideRosterNavbar" onClick={handleClick}>
                Show/Hide UAV Roster
              </a>
              <a
                id="cameraView"
                onClick={() => {
                  navigate("/camera");
                  handleClick();
                }}
              >
                Camera view
              </a>
            </div>
          </div>
          <div className="dropdown">
            <button className="dropbtn">
              File
              <i className="fa fa-caret-down"></i>
            </button>
            <div name="otrotest" className="dropdown-content">
              <label id="menuopenmission" htmlFor="openMissionNavbar">
                Open Mision
              </label>
              <input
                accept=".yaml, .plan, .waypoint, .kml"
                type="file"
                multiple={false}
                style={{ display: "none" }}
                id="openMissionNavbar"
                onChange={readFile}
              />
              <a
                id="Clear mission"
                onClick={() => {
                  clearmission();
                  handleClick();
                }}
              >
                Clear Mission
              </a>
              <a
                id="editmission"
                onClick={() => {
                  navigate("/mission");
                  handleClick();
                }}
              >
                Edit mission
              </a>
            </div>
          </div>
          <div className="dropdown">
            <button className="dropbtn">
              Mission
              <i className="fa fa-caret-down"></i>
            </button>
            <div name="otrotest" className="dropdown-content">
              <a
                id="loadMissionNavbar"
                onClick={() => {
                  rosContex.loadMission();
                  handleClick();
                }}
              >
                Load Mission
              </a>
              <a
                id="commandMissionNavbar"
                onClick={() => {
                  rosContex.commandMission();
                  handleClick();
                }}
              >
                Command Mission
              </a>
            </div>
          </div>
          <div className="dropdown">
            <button className="dropbtn">
              Report
              <i className="fa fa-caret-down"></i>
            </button>
          </div>
        </Toolbar>
      </Container>
    </AppBar>
  );
};
