import React, { useState, useEffect } from 'react';
import { useSelector } from 'react-redux';
import { Paper } from '@mui/material';
import { makeStyles } from 'tss-react/mui';


import MapView from '../map/core/MapView';
import Navbar from '../components/layout/Navbar';
import { Menu } from '../components/layout/Menu';
import MapMissionsCreate from '../map/draw/MapMissionsCreate';
import MapMissions3D from '../map/mission/MapMissions3D';
import MapPositions from '../map/devices/MapPositions';
import MapMarkers from '../map/devices/MapMarkers';

import { RosControl } from '../components/commands/RosControl';
import MissionPanel from '../components/mission/MissionPanel';
import MissionElevation from '../components/mission/MissionElevation';
import SaveFile from '../components/ui/SaveFile';
import MapScale from '../map/controls/MapScale';
import MapDefaultCamera from '../map/controls/MapDefaultCamera';
import MapMarkers3D from '../map/devices/MapMarkers3D';

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
    height: '50vh',
    width: 'calc(100% - 560px)',
    margin: '0px',
    zIndex: 3,
  },
}));
const showToast = (type, description) => {
  setList([...list, toastProperties]);
};

const MissionPageTest = () => {
  const { classes } = useStyles();
  const [Opensave, setOpenSave] = useState(false);

  const positions = useSelector((state) => state.session.positions);
  const sessionmarkers = useSelector((state) => state.session.markers);

  const [filteredPositions, setFilteredPositions] = useState([]);
  const [markers, setmarkers] = useState([]);

  useEffect(() => {
    setFilteredPositions(Object.values(positions));
  }, [positions]);
  useEffect(() => {
    setmarkers(sessionmarkers);
  }, [sessionmarkers]);

  return (
    <div className={classes.root}>
      <RosControl notification={showToast}>
          <Navbar />
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
            <MapView>
              <MapMarkers markers={markers} />
              <MapDefaultCamera />
              <MapMissionsCreate />
              <MapMissions3D />
              <MapMarkers3D />
              <MapPositions positions={filteredPositions} onClick={null} selectedPosition={null} showStatus />
            </MapView>
            <MapScale />
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
              <Paper square>
                <MissionElevation />
              </Paper>
            </div>
          </div>
          {Opensave && <SaveFile SetOpenSave={setOpenSave} />}
      </RosControl>
    </div>
  );
};

export default MissionPageTest;
