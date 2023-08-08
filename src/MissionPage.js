import './MainPage.css';
import React from 'react'
import MapView from './Mapview/Mapview'
import { Navbar } from './components/Navbar';
import { Menu } from './components/Menu';
import MapMissionsCreate from './Mapview/draw/MapMissionsCreate';
import RoutesList from './components/RoutesList';
import {   Divider, Typography, IconButton,Drawer,Paper, Toolbar,} from '@mui/material';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import UploadFileIcon from '@mui/icons-material/UploadFile';
import SaveAltIcon from '@mui/icons-material/SaveAlt';
import DeleteIcon from '@mui/icons-material/Delete';
import { useNavigate } from 'react-router-dom';
import makeStyles from '@mui/styles/makeStyles';
import MapMissions from './Mapview/MapMissions';
import { RosControl } from './components/RosControl';


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
    height: `calc(100% - 88px)`,
    width: '560px',
    margin: '0px',
    zIndex: 3,
  },
  middleStyle: {
    flex: 1,
    display: 'grid',
  },
  toolbar:{
    display: 'flex',
    gap: '10px 10px',
    height: '30px',
    borderBottom: "3px solid rgb(212, 212, 212)",
},
list: {
  maxHeight: `calc(100vh - 152px)`,
  overflowY: 'auto',
},
  title: {
    flexGrow: 1,
  },
  fileInput: {
    display: 'none',
  },
}));
const showToast = (type,description)=> {
  switch(type) {
    case 'success':
      toastProperties = {
        id: list.length+1,
        title: 'Success',
        description: description,
        backgroundColor: '#5cb85c'
      }
      break;
    case 'danger':
      toastProperties = {
        id: list.length+1,
        title: 'Danger',
        description: description,
        backgroundColor: '#d9534f'
      }
      break;
    case 'info':
      toastProperties = {
        id: list.length+1,
        title: 'Info',
        description: description,
        backgroundColor: '#5bc0de'
      }
      break;
    case 'warning':
      toastProperties = {
        id: list.length+1,
        title: 'Warning',
        description: description,
        backgroundColor: '#f0ad4e'
      }
      break;
    default:
      toastProperties = [];
    }
    setList([...list, toastProperties]);
}

const MissionPage = () => {
  const classes = useStyles();
  const navigate = useNavigate();

  return (
    <div className={classes.root} >
      <RosControl notification={showToast}>
        <Navbar/>
        <Menu/>
        <MapView>
          <MapMissionsCreate/>
          <MapMissions/>
        </MapView>
        <div className={classes.sidebarStyle}>
          <div className={classes.middleStyle}>
            <Paper square className={classes.contentListStyle} >          
              <Toolbar className={classes.toolbar}>
                <IconButton edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
                  <ArrowBackIcon />
                </IconButton>
                <Typography variant="h6" className={classes.title}>Mission Task</Typography>
                <IconButton onClick={() => console.log("save")} >
                      <SaveAltIcon />
                </IconButton>
                <IconButton onClick={() => console.log("remove")} >
                      <DeleteIcon />
                </IconButton>
                <label htmlFor="upload-gpx">
                  <input accept=".gpx" id="upload-gpx" type="file" className={classes.fileInput} />
                  <IconButton edge="end" component="span" onClick={() => {}}>
                    <UploadFileIcon />
                  </IconButton>
                </label>
              </Toolbar>
                <div className={classes.list}>
                <RoutesList RouteSelect />
                </div>
                
            </Paper>
          </div>
        </div>

      </RosControl>
    </div>
  )
}

export default MissionPage