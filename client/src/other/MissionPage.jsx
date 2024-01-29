import React, { useState, useEffect } from 'react';
import MapView from '../Mapview/MapView';
import { Navbar } from '../components/Navbar';
import { Menu } from '../components/Menu';
import MapMissionsCreate from '../Mapview/draw/MapMissionsCreate';
import MapPositions from '../Mapview/MapPositions';
import { useSelector } from 'react-redux';

import { Paper } from '@mui/material';

import makeStyles from '@mui/styles/makeStyles';
import { RosControl } from '../components/RosControl';
import { MissionController } from '../components/MissionController';
import MissionPanel from '../components/MissionPanel';
import MissionElevation from '../components/MissionElevation';
import SaveFile from '../components/SaveFile';
import MapScale from '../Mapview/MapScale';

const useStyles = makeStyles((theme) => ({
  root: {
    height: '100%',
  },
  sidebarStyle: {
    display: 'flex',
    flexDirection: 'column',
    position: 'fixed',
    left: 0,
    top: '88px',
    height: `calc(100% - 95px)`,
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
    height: `30vh`,
    width: `calc(100% - 560px)`,
    margin: '0px',
    zIndex: 3,
  },
}));
const showToast = (type, description) => {
  switch (type) {
    case 'success':
      toastProperties = {
        id: list.length + 1,
        title: 'Success',
        description: description,
        backgroundColor: '#5cb85c',
      };
      break;
    case 'danger':
      toastProperties = {
        id: list.length + 1,
        title: 'Danger',
        description: description,
        backgroundColor: '#d9534f',
      };
      break;
    case 'error':
      toastProperties = {
        id: list.length + 1,
        title: 'Danger',
        description: description,
        backgroundColor: '#d9534f',
      };
      break;
    case 'info':
      toastProperties = {
        id: list.length + 1,
        title: 'Info',
        description: description,
        backgroundColor: '#5bc0de',
      };
      break;
    case 'warning':
      toastProperties = {
        id: list.length + 1,
        title: 'Warning',
        description: description,
        backgroundColor: '#f0ad4e',
      };
      break;
    default:
      toastProperties = [];
  }
  setList([...list, toastProperties]);
};

const MissionPage = () => {
  const classes = useStyles();
  const [Opensave, setOpenSave] = useState(false);

  const positions = useSelector((state) => state.data.positions);

  const [filteredPositions, setFilteredPositions] = useState([]);

  useEffect(() => {
    setFilteredPositions(Object.values(positions));
  }, [positions]);

  return (
    <div className={classes.root}>
      <MissionController>
        <RosControl notification={showToast}>
          <Navbar />
          <Menu />
          <div
            style={{
              position: 'relative',
              width: '100%',
              height: `calc(100vh - 95px)`,
            }}
          >
            <div
              style={{
                display: 'inline-block',
                position: 'relative',
                width: '560px',
                height: '100%',
              }}
            ></div>
            <div
              style={{
                display: 'inline-block',
                position: 'relative',
                width: `calc(100vw - 575px)`,
                height: '100%',
              }}
            >
              <MapView>
                <MapMissionsCreate />
                <MapPositions
                  positions={filteredPositions}
                  onClick={null}
                  selectedPosition={null}
                  showStatus
                />
              </MapView>
              <MapScale />
            </div>
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
      </MissionController>
    </div>
  );
};

export default MissionPage;
