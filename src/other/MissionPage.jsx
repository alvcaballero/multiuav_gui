//import "./MainPage.css";
import React, { useContext, useState } from 'react';
import MapView from '../Mapview/MapView';
import { Navbar } from '../components/Navbar';
import { Menu } from '../components/Menu';
import MapMissionsCreate from '../Mapview/draw/MapMissionsCreate';

import { Divider, Typography, IconButton, Drawer, Paper, Toolbar } from '@mui/material';

import makeStyles from '@mui/styles/makeStyles';
import { RosControl, RosContext } from '../components/RosControl';
import { MissionPanel } from '../components/MissionPanel';
import MissionElevation from '../components/MissionElevation';
import { SaveFile } from '../components/SaveFile';

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

  return (
    <div className={classes.root}>
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
            </MapView>
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
    </div>
  );
};

export default MissionPage;
