import './MainPage.css';
import React from 'react'
import MapView from './Mapview/Mapview'
import { Navbar } from './components/Navbar';
import MapMissionsCreate from './Mapview/draw/MapMissionsCreate';
import Drawer from '@mui/material/Drawer';
import RoutesList from './components/RoutesList';
import {
  Divider, Typography, IconButton, Toolbar,
} from '@mui/material';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import UploadFileIcon from '@mui/icons-material/UploadFile';
import { useNavigate } from 'react-router-dom';
import makeStyles from '@mui/styles/makeStyles';


const useStyles = makeStyles(() => ({
  root: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
  },
  content: {
    flexGrow: 1,
    overflow: 'hidden',
    display: 'flex',
    //flexDirection: 'row',
    flexDirection: 'column-reverse',
  },
  drawerPaper: {
    position: 'relative',
    zIndex: 0,
    width: "30%",
  },
  mapContainer: {
    flexGrow: 1,
  },
  title: {
    flexGrow: 1,
  },
  fileInput: {
    display: 'none',
  },
}));

const MissionPage = () => {
  const classes = useStyles();
  const navigate = useNavigate();

  return (
    <div className={classes.root} >
      <div className={classes.content}>
        <Navbar  />
        <Drawer anchor='left' variant="permanent" classes={{ paper: classes.drawerPaper }}  >
          <Toolbar>
          <IconButton edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
              <ArrowBackIcon />
            </IconButton>
            <Typography variant="h6" className={classes.title}>sharedGeofences</Typography>
            <label htmlFor="upload-gpx">
              <input accept=".gpx" id="upload-gpx" type="file" className={classes.fileInput} />
              <IconButton edge="end" component="span" onClick={() => {}}>
                <UploadFileIcon />
              </IconButton>
              </label>
          </Toolbar>
          <Divider/>
          <RoutesList RouteSelect/>

        </Drawer>
        <div className={classes.mapContainer}>
          <MapView>
            <MapMissionsCreate/>
          </MapView>
        </div>

      </div>
    </div>
  )
}

export default MissionPage