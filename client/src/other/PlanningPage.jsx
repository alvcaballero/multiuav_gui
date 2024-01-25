//import "./MainPage.css";
import React, { useContext, useState, useEffect } from 'react';

import MapView from '../Mapview/MapView';
import { Navbar } from '../components/Navbar';
import { Menu } from '../components/Menu';
import MapMissionsCreate from '../Mapview/draw/MapMissionsCreate';
import SelectField from '../common/components/SelectField';
import ExpandMore from '@mui/icons-material/ExpandMore';
import palette from '../common/palette';
import BaseList from '../components/BaseList';

import { useSelector } from 'react-redux';

import {
  Paper,
  IconButton,
  Box,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Divider,
  Button,
  TextField,
  Toolbar,
  Typography,
} from '@mui/material';

import makeStyles from '@mui/styles/makeStyles';
import { RosControl } from '../components/RosControl';
import { MissionController } from '../components/MissionController';
import MissionPanel from '../components/MissionPanel';
import MissionElevation from '../components/MissionElevation';
import SaveFile from '../components/SaveFile';
import MapMarkers from '../Mapview/MapMarkers';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
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
  toolbar: {
    display: 'flex',
    gap: '10px 10px',
    height: '30px',
    borderBottom: '3px solid rgb(212, 212, 212)',
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
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
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

const PlanningPage = () => {
  const classes = useStyles();
  const [Opensave, setOpenSave] = useState(false);

  const positions = useSelector((state) => state.data.positions);
  const sessionmarkers = useSelector((state) => state.session.markers);

  const [filteredPositions, setFilteredPositions] = useState([]);
  const [markers, setmarkers] = useState([]);
  const [SendTask, setSendTask] = useState({
    id: 1234,
    name: 'no mission',
    objetivo: 0,
    loc: [],
    meteo: [],
    setting: [],
  });
  useEffect(() => {
    setmarkers(sessionmarkers);
  }, [sessionmarkers]);

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
                <MapMarkers markers={markers} />
              </MapView>
            </div>
          </div>

          <div className={classes.sidebarStyle}>
            <div className={classes.middleStyle}>
              <Paper square>
                <Toolbar className={classes.toolbar}>
                  <IconButton edge='start' sx={{ mr: 2 }} onClick={() => navigate(-1)}>
                    <ArrowBackIcon />
                  </IconButton>
                  <Typography variant='h6' className={classes.title}>
                    Mission Elements
                  </Typography>
                </Toolbar>
                <div className={classes.list}>
                  <Box
                    style={{
                      display: 'flex',
                      gap: '10px',
                      flexDirection: 'column',
                      margin: '20px',
                    }}
                  >
                    <TextField
                      required
                      label='id'
                      type='number'
                      variant='standard'
                      value={SendTask.id ? SendTask.id : 123}
                      onChange={(event) => setSendTask({ ...SendTask, id: event.target.value })}
                    />
                    <TextField
                      required
                      label='Name Mission'
                      variant='standard'
                      value={SendTask.name ? SendTask.name : ' '}
                      onChange={(event) => setSendTask({ ...SendTask, name: event.target.value })}
                    />
                    <SelectField
                      emptyValue={null}
                      value={SendTask.objetivo}
                      onChange={(e) =>
                        setSendTask({
                          ...SendTask,
                          objetivo: e.target.value,
                        })
                      }
                      data={[
                        { id: 0, name: 'Localizacion de inidencia' },
                        { id: 1, name: 'zona de aves' },
                        { id: 2, name: 'Vegetacion' },
                      ]}
                      label={'objetivo'}
                      style={{ display: 'inline', width: '200px' }}
                    />

                    <Accordion>
                      <AccordionSummary expandIcon={<ExpandMore />}>
                        <Typography>localizacion incidentia</Typography>
                      </AccordionSummary>
                      <AccordionDetails className={classes.details}></AccordionDetails>
                    </Accordion>
                    <Accordion>
                      <AccordionSummary expandIcon={<ExpandMore />}>
                        <Typography>Meteo</Typography>
                      </AccordionSummary>
                      <AccordionDetails className={classes.details}></AccordionDetails>
                    </Accordion>
                    <Divider></Divider>
                    <Accordion>
                      <AccordionSummary expandIcon={<ExpandMore />}>
                        <Typography>Mission Settings</Typography>
                      </AccordionSummary>
                      <AccordionDetails className={classes.details}>
                        <BaseList mission={SendTask} setmission={setSendTask} />
                      </AccordionDetails>
                    </Accordion>
                    <Accordion>
                      <AccordionSummary expandIcon={<ExpandMore />}>
                        <Typography>Elementos Interes</Typography>
                      </AccordionSummary>
                      <AccordionDetails className={classes.details}>
                        <BaseList mission={SendTask} setmission={setSendTask} />
                      </AccordionDetails>
                    </Accordion>
                  </Box>
                </div>
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

export default PlanningPage;
