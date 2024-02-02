import React, { useState, useEffect } from 'react';

import { useDispatch, useSelector } from 'react-redux';

import {
  Paper,
  IconButton,
  Box,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Button,
  TextField,
  Toolbar,
  Tabs,
  Divider,
  Tab,
  Typography,
} from '@mui/material';
import ExpandMore from '@mui/icons-material/ExpandMore';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import { TabPanel, TabList, TabContext } from '@mui/lab';

import makeStyles from '@mui/styles/makeStyles';
import { useNavigate } from 'react-router-dom';

import { sessionActions } from '../store';

import MapView from '../Mapview/MapView';
import { Navbar } from '../components/Navbar';
import { Menu } from '../components/Menu';
import MapMissions from '../Mapview/MapMissions';
import SelectField from '../common/components/SelectField';
import palette from '../common/palette';
import BaseList from '../components/BaseList';
import BaseSettings from '../components/BaseSettings';
import { RosControl } from '../components/RosControl';
import { MissionController } from '../components/MissionController';
import MissionElevation from '../components/MissionElevation';
import SaveFile from '../components/SaveFile';
import MapMarkersCreate from '../Mapview/draw/MapMarkersCreate';
import MapScale from '../Mapview/MapScale';
import ElementList from '../components/ElementList';
import { useEffectAsync } from '../reactHelper';
import { SettingsOutlined } from '@mui/icons-material';

const useStyles = makeStyles((theme) => ({
  root: {
    margin: '0',
    height: '100vh',
  },
  sidebarStyle: {
    display: 'flex',
    flexDirection: 'column',
    position: 'fixed',
    left: 0,
    top: '88px',
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
  setList([...list, toastProperties]);
};

const PlanningPage = () => {
  const classes = useStyles();
  const navigate = useNavigate();
  const dispatch = useDispatch();
  const [Opensave, setOpenSave] = useState(false);
  const [showTitles, setShowTitles] = useState(true);
  const [showLines, setShowLines] = useState(true);
  const [moveMarkers, SetMoveMarkers] = useState(false);
  const [SelectMarkers, SetSelectMarkers] = useState(false);
  const [CreateMarkers, SetCreateMarkers] = useState(false);
  const [descriptionObject, Setdescription] = useState('select elements in the map');

  const positions = useSelector((state) => state.data.positions);
  const sessionmarkers = useSelector((state) => state.session.markers);
  const sessionplanning = useSelector((state) => state.session.planning);

  const [filteredPositions, setFilteredPositions] = useState([]);
  const [markers, setMarkers] = useState(sessionmarkers);
  const [SendTask, setSendTask] = useState(sessionplanning);
  const [tabValue, setTabValue] = useState('2');
  const [setting, setsetting] = useState('');

  const TabHandleChange = (event, newTabValue) => {
    setTabValue(newTabValue);
    newTabValue == 2 || newTabValue == 1 ? setShowTitles(true) : setShowTitles(false);
    newTabValue == 1 ? setShowLines(true) : setShowLines(false);
    newTabValue == 1 ? SetMoveMarkers(true) : SetMoveMarkers(false);
  };

  const setMarkersBase = (value) => {
    setMarkers({ ...markers, bases: value });
    setSendTask((oldSendtask) => {
      let auxbases = value.map((base) => {
        if (!base.device.hasOwnProperty('id')) {
          base.device = oldSendtask.settings.device;
          base.missions = oldSendtask.settings.mission;
        }
        return base;
      });
      return { ...oldSendtask, bases: auxbases };
    });
  };
  const setMarkersElements = (value) => {
    setMarkers({ ...markers, elements: value });
  };
  const SetMapMarkers = (value) => {
    setMarkers(value);
  };
  const setLocations = (value) => {
    setSendTask({ ...SendTask, loc: value });
  };
  const managepoints = (old, point, objetivo) => {
    if (objetivo == 0 || objetivo == 0) {
      if (old.length == 0) {
        old.push({ type: 'powerTower', name: 'Elements', linea: true, items: [point] });
      } else {
        const samerute = old.find((element) => element.items[0].groupId == point.groupId);
        if (samerute === undefined) {
          old.push({ type: 'powerTower', name: 'Elements', linea: true, items: [point] });
        } else {
          old[+old.length - +1].items.push(point);
        }
      }
    }
    if (objetivo == 3 || objetivo == 1) {
      old.push({ type: 'powerTower', name: 'Elements', linea: true, items: [point] });
    }

    return old;
  };
  const addLocations = (value) => {
    console.log(value);
    setSendTask((oldSendtask) => {
      let auxloc = JSON.parse(JSON.stringify(oldSendtask.loc));
      return { ...oldSendtask, loc: managepoints(auxloc, value, oldSendtask.objetivo) };
    });
  };

  useEffectAsync(async () => {
    const response = await fetch(`/api/planning/missionparam/${SendTask.objetivo}`);
    if (response.ok) {
      setsetting(await response.json());
    } else {
      throw Error(await response.text());
    }
  }, [SendTask.objetivo]);

  useEffect(() => {
    console.log(setting);
    if (setting.hasOwnProperty('mission')) {
      setSendTask({ ...SendTask, setting: setting });
    }
  }, [setting]);

  useEffect(() => {
    tabValue == 2 && SendTask.objetivo != 3 ? SetSelectMarkers(true) : SetSelectMarkers(false);
    tabValue == 2 && SendTask.objetivo == 3 ? SetCreateMarkers(true) : SetCreateMarkers(false);
  }, [tabValue, SendTask.objetivo]);

  useEffect(() => {
    setMarkers(sessionmarkers);
  }, [sessionmarkers]);

  useEffect(() => {
    setSendTask(sessionplanning);
  }, [sessionplanning]);

  useEffect(() => {
    setFilteredPositions(Object.values(positions));
  }, [positions]);
  useEffect(() => {
    dispatch(sessionActions.updateMarker(markers));
  }, [markers]);

  useEffect(() => {
    dispatch(sessionActions.updatePlanning(SendTask));
  }, [SendTask]);

  return (
    <div className={classes.root}>
      <MissionController>
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
              <MapMissions />
              <MapMarkersCreate
                markers={markers}
                showTitles={showTitles}
                showLines={showLines}
                moveMarkers={moveMarkers}
                setMarkers={SetMapMarkers}
                SelectItems={SelectMarkers}
                CreateItems={CreateMarkers}
                setLocations={addLocations}
              />
            </MapView>
            <MapScale />
          </div>

          <div className={classes.sidebarStyle}>
            <div className={classes.middleStyle}>
              <Paper square>
                <Toolbar className={classes.toolbar}>
                  <IconButton edge='start' sx={{ mr: 2 }} onClick={() => navigate(-1)}>
                    <ArrowBackIcon />
                  </IconButton>
                  <Typography variant='h6' className={classes.title}>
                    Planning mission
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
                    <TabContext value={tabValue}>
                      <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
                        <TabList onChange={TabHandleChange} aria-label='lab API tabs example'>
                          <Tab label='Elements' value='1' />
                          <Tab label='Planning' value='2' />
                          <Tab label='Settings' value='3' />
                        </TabList>
                      </Box>
                      <TabPanel value='1'>
                        <div className={classes.details}>
                          <Accordion>
                            <AccordionSummary expandIcon={<ExpandMore />}>
                              <Typography>Base elements</Typography>
                            </AccordionSummary>
                            <AccordionDetails className={classes.details}>
                              <BaseList markers={markers.bases} setMarkers={setMarkersBase} />
                            </AccordionDetails>
                          </Accordion>
                          <Divider />
                          <Accordion>
                            <AccordionSummary expandIcon={<ExpandMore />}>
                              <Typography>Interest Elements</Typography>
                            </AccordionSummary>
                            <AccordionDetails className={classes.details}>
                              <ElementList
                                markers={markers.elements}
                                setMarkers={setMarkersElements}
                              />
                            </AccordionDetails>
                          </Accordion>
                        </div>
                      </TabPanel>
                      <TabPanel value='2'>
                        <Box className={classes.details}>
                          <TextField
                            required
                            fullWidth
                            label='id'
                            type='number'
                            variant='standard'
                            value={SendTask.id ? SendTask.id : 123}
                            onChange={(event) =>
                              setSendTask({ ...SendTask, id: event.target.value })
                            }
                          />
                          <TextField
                            required
                            fullWidth
                            label='Name Mission'
                            variant='standard'
                            value={SendTask.name ? SendTask.name : ' '}
                            onChange={(event) =>
                              setSendTask({ ...SendTask, name: event.target.value })
                            }
                          />
                          <SelectField
                            emptyValue={null}
                            fullWidth={true}
                            label={'objetive'}
                            value={SendTask.objetivo}
                            endpoint={'/api/planning/missionstype'}
                            keyGetter={(it) => it.id}
                            titleGetter={(it) => it.name}
                            onChange={(e, items) => {
                              setSendTask({ ...SendTask, objetivo: e.target.value });
                              console.log(items[e.target.value]);
                              Setdescription(items[e.target.value].description);
                            }}
                          />

                          <Accordion>
                            <AccordionSummary expandIcon={<ExpandMore />}>
                              <Typography>Points to inspect </Typography>
                            </AccordionSummary>
                            <AccordionDetails className={classes.details}>
                              <div className={classes.details}>
                                <Typography>{descriptionObject}</Typography>
                                <ElementList markers={SendTask.loc} setMarkers={setLocations} />
                              </div>
                            </AccordionDetails>
                          </Accordion>
                          <Divider />
                          <Accordion>
                            <AccordionSummary expandIcon={<ExpandMore />}>
                              <Typography>Meteo</Typography>
                            </AccordionSummary>
                            <AccordionDetails className={classes.details}>
                              <Typography>Meteo</Typography>
                            </AccordionDetails>
                          </Accordion>
                        </Box>
                      </TabPanel>
                      <TabPanel value='3'>
                        <div className={classes.details}>
                          <BaseSettings markers={markers.bases} setMarkers={setMarkersBase} />

                          <Box textAlign='center'>
                            <Button
                              variant='contained'
                              size='large'
                              sx={{ width: '80%', flexShrink: 0 }}
                              style={{ marginTop: '15px' }}
                            >
                              Plannig
                            </Button>
                          </Box>
                        </div>
                      </TabPanel>
                    </TabContext>
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
