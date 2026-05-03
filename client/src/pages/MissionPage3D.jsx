import React, { useState, useEffect, Fragment } from 'react';
import { useSelector } from 'react-redux';
import { Paper, Tab, Tabs } from '@mui/material';
import { makeStyles } from 'tss-react/mui';

import { Navbar2 } from '../components/layout/Navbar2';
import { Menu } from '../components/layout/Menu';

import { RosControl } from '../components/commands/RosControl';
import MissionPanel from '../components/mission/MissionPanel';
import MissionElevation from '../components/mission/MissionElevation';
import SaveFile from '../components/ui/SaveFile';


import R3FCanvas from '../scene3d/core/R3FCanvas';
import R3FMission from '../scene3d/scene/R3FMission';
import R3DMarkers from '../scene3d/scene/R3DMarkers';
import R3FDevices from '../scene3d/scene/R3FDevices';

const useStyles = makeStyles()((theme) => ({
  root: {
    margin: '0',
    height: '100vh',
  },
  sidebarStyle: {
    display: 'flex',
    flexDirection: 'column',
    position: 'fixed',
    left: 0,
    top: theme.dimensions.navbarHeight,
    height: 'calc(100% - 95px)',
    width: '560px',
    margin: '0px',
    zIndex: 3,
  },
  middleStyle: {
    flex: 1,
    display: 'grid',
  },
  panelElevation: {
    display: 'flex',
    flexDirection: 'column',
    position: 'fixed',
    right: 0,
    bottom: 0,
    height: '30vh',
    width: 'calc(100% - 560px)',
    margin: '0px',
    zIndex: 3,
  },
}));
const showToast = (type, description) => {
  setList([...list, toastProperties]);
};

const MissionPage3D = () => {
  const { classes } = useStyles();
  const tabIndex = 0;

  const [Opensave, setOpenSave] = useState(false);

  const positions = useSelector((state) => state.session.positions);
  const routes = useSelector((state) => state.mission.route);
  const sessionmarkers = useSelector((state) => state.session.markers);
  const origin3d = useSelector((state) => state.session.scene3d.origin);


  const [markers, setmarkers] = useState([]);



  const [filteredPositions, setFilteredPositions] = useState([]);

  const elements = [
    { type: "windturbine", pos: [10, 10, 0] },
    { type: "base", pos: [0, 0, 0] },
    { type: "drone", pos: [1, 1, 1] }
  ]

  useEffect(() => {
    setFilteredPositions(Object.values(positions));
  }, [positions]);

  useEffect(() => {
    setmarkers(sessionmarkers);
  }, [sessionmarkers]);

  const tabs = (
    <>
      <Tabs value={tabIndex} onChange={(_, index) => navigate(`/robot/${id}/${index}`)} style={{ flexGrow: 1 }}>
        <Tab label="Viz" />
        <Tab label="Imagery" />
        <Tab label="Stats" />
        <Tab label="Report" />
      </Tabs>
    </>
  );

  return (
    <div className={classes.root}>
      <RosControl notification={showToast}>
          <Navbar2 tabs={tabs} />
          <Menu />
          <div
            style={{
              float: 'right',
              width: 'calc(100% - 560px)',
              height: 'calc(70vh - 95px)',
              right: '0px',
              margin: 'auto',
            }}
          >
            <R3FCanvas>
              <R3FMission routes={routes} />
              <R3DMarkers elements={markers} />
              <R3FDevices />
            </R3FCanvas>

          </div>

          <div className={classes.sidebarStyle}>
            <div className={classes.middleStyle}>
              <Paper square>
                <MissionPanel SetOpenSave={setOpenSave} />
              </Paper>
            </div>
          </div>
          <div className={classes.panelElevation}>
            <div className={classes.middleStyle}>
              <Paper square sx={{ height: '100%' }}>
                <MissionElevation />
              </Paper>
            </div>
          </div>
          {Opensave && <SaveFile SetOpenSave={setOpenSave} />}
      </RosControl>
    </div>
  );
};

export default MissionPage3D;
