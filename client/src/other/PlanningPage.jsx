import React, { useState, useEffect } from 'react';

import { useDispatch, useSelector } from 'react-redux';
import YAML from 'yaml';

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
  Divider,
  Tab,
  Typography,
  Switch,
} from '@mui/material';
import ExpandMore from '@mui/icons-material/ExpandMore';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import { TabPanel, TabList, TabContext } from '@mui/lab';
import SaveAltIcon from '@mui/icons-material/SaveAlt';
import DeleteIcon from '@mui/icons-material/Delete';

import makeStyles from '@mui/styles/makeStyles';
import { useNavigate } from 'react-router-dom';
import { missionActions, sessionActions } from '../store'; // here update device action with position of uav for update in map

import MapView, { map } from '../Mapview/MapView';
import Navbar from '../components/Navbar';
import { Menu } from '../components/Menu';
import MapMissions from '../Mapview/MapMissions';
import SelectField from '../common/components/SelectField';
import BaseList from '../components/BaseList';
import BaseSettings from '../components/BaseSettings';
import { RosControl } from '../components/RosControl';
import { MissionController } from '../components/MissionController';
import MissionElevation from '../components/MissionElevation';
import MapMarkersCreate from '../Mapview/draw/MapMarkersCreate';
import MapScale from '../Mapview/MapScale';
import ElementList from '../components/ElementList';
import { useEffectAsync, useCatch } from '../reactHelper';
import SelectList from '../components/SelectList';
import MapDefaultCamera from '../Mapview/MapDefaultCamera';
import UploadButtons from '../components/uploadButton';

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
    maxHeight: 'calc(100vh - 152px)',
    overflowY: 'auto',
    display: 'flex',
    gap: '10px',
    flexDirection: 'column',
    margin: '20px',
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
  panelButton: {
    width: '80%',
    flexShrink: 0,
    marginTop: '15px',
  },
  mapContainer: {
    float: 'right',
    width: 'calc(100% - 560px)',
    height: 'calc(70vh - 95px)',
    right: '0px',
    margin: 'auto',
  },
}));
const showToast = (type, description) => {
  console.log(type + description);
};

const PlanningPage = () => {
  const classes = useStyles();
  const navigate = useNavigate();
  const dispatch = useDispatch();
  const [showTitles, setShowTitles] = useState(true);
  const [showLines, setShowLines] = useState(false);
  const [moveMarkers, SetMoveMarkers] = useState(false);
  const [SelectMarkers, SetSelectMarkers] = useState(false);
  const [CreateMarkers, SetCreateMarkers] = useState(false);
  const [requestPlanning, SetRequestPlanning] = useState(100);
  const myhostname = `${window.location.hostname}`;

  const sessionmarkers = useSelector((state) => state.session.markers);
  const sessionplanning = useSelector((state) => state.session.planning);

  const [markers, setMarkers] = useState(sessionmarkers);
  const [SendTask, setSendTask] = useState(sessionplanning);
  const [auxobjetive, setauxobjetive] = useState(-1);
  const [tabValue, setTabValue] = useState('2');
  const [notification, setNotification] = useState('');
  const [checked, setChecked] = useState(true);

  const SendPlanning = useCatch(async () => {
    console.log('send planning');
    let myTask = {};
    myTask.id = SendTask.id;
    myTask.name = SendTask.name;
    myTask.case = SendTask.objetivo.case;
    myTask.meteo = SendTask.meteo;
    myTask.locations = SendTask.loc.map((group) => {
      let items = group.items.map((element) => ({
        latitude: element.latitude,
        longitude: element.longitude,
      }));
      return { name: group.name, items };
    });

    let listDevices = SendTask.bases.map((setting) => setting.devices.id);
    let deviceRepeat = listDevices.some((value, index, list) => !(list.indexOf(value) === index));
    setNotification('');
    if (deviceRepeat) {
      setNotification('the device is repeated please change');
      return null;
    }
    if (SendTask.loc.length === 0) {
      setNotification('No elements to inspection');
      return null;
    }

    let devices = [];
    const response = await fetch('/api/devices');
    if (response.ok) {
      devices = await response.json();
    } else {
      throw Error(await response.text());
    }
    myTask.devices = SendTask.bases.map((setting, index) => {
      let mySetting = JSON.parse(JSON.stringify(setting));
      let myDevice = devices.find((device) => device.id == setting.devices.id);
      delete mySetting.devices;
      mySetting.id = myDevice.name;
      mySetting.category = myDevice.category;
      mySetting.settings.base = Object.values(markers.bases[index]);
      mySetting.settings.landing_mode = 2;
      return mySetting;
    });
    console.log(myTask);
    const response1 = await fetch(`http://${myhostname}:8004/mission_request`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(myTask),
    });
    if (response1.ok) {
      const response2 = await response1.json();
      console.log(response2);
      SetRequestPlanning(0);
    } else {
      throw Error(await response.text());
    }
    return true;
  });
  const DeleteMission = () => {
    console.log('uno');
  };
  const SavePlanning = (value) => {
    let fileData = YAML.stringify(value);
    const blob = new Blob([fileData], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.download = `${value.name}.yaml`;
    link.href = url;
    link.click();
  };
  const MissionTask = async () => {
    const myTask = {};
    myTask.id = SendTask.id;
    myTask.name = SendTask.name;
    myTask.objetivo = SendTask.objetivo.id;
    myTask.meteo = SendTask.meteo;
    myTask.locations = SendTask.loc.map((group) => {
      let items = group.items.map((element) => ({
        latitude: element.latitude,
        longitude: element.longitude,
      }));
      return { name: group.name, items };
    });
    console.log(myTask);

    const response = await fetch('/api/missions/sendTask', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(myTask),
    });
    if (response.ok) {
      const response2 = await response.json();
      console.log(response2);
    } else {
      throw Error(await response.text());
    }
  };
  const goToBase = (index) => {
    map.flyTo({
      center: [markers.bases[index].longitude, markers.bases[index].latitude],
      zoom: 16,
    });
  };

  /*
   * https://www.youtube.com/watch?v=K3SshoCXC2g
   */
  const readFile = (e) => {
    const file = e.target.files[0];
    if (!file) return;

    const fileReader = new FileReader();
    fileReader.readAsText(file);
    fileReader.onload = () => {
      let myTask = YAML.parse(fileReader.result);
      setMarkers((oldmarkers) => ({
        ...oldmarkers,
        bases: myTask.markersbase,
        elements: myTask.elements,
      }));

      myTask.hasOwnProperty('markersbase') ? delete myTask.markersbase : null;
      myTask.hasOwnProperty('elements') ? delete myTask.elements : null;
      setSendTask(myTask);
    };
    fileReader.onerror = () => {
      console.log(fileReader.error);
    };
  };

  const setDefaultPlanning = async (value) => {
    console.log('Setdefault');
    const response = await fetch('api/planning/setDefault', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(value),
    });
    if (response.ok) {
      const response2 = await response.json();
    } else {
      throw Error(await response.text());
    }
  };
  const setBaseSettings = (element) => {
    setSendTask({ ...SendTask, bases: element });
  };
  const TabHandleChange = (event, newTabValue) => {
    setTabValue(newTabValue);
    newTabValue == 2 || newTabValue == 1 || newTabValue == 3 ? setShowTitles(true) : setShowTitles(false);
    newTabValue == 1 ? setShowLines(true) : setShowLines(false);
    newTabValue == 1 ? SetMoveMarkers(true) : SetMoveMarkers(false);
  };

  const setMarkersBase = (value, meta = {}) => {
    setMarkers({ ...markers, bases: value });
    if (meta.hasOwnProperty('meth')) {
      if (meta.meth == 'add') {
        setSendTask((oldSendtask) => {
          let auxbases = JSON.parse(JSON.stringify(oldSendtask.bases));
          let auxconfig = {};
          Object.keys(oldSendtask.settings).forEach((key) => {
            auxconfig[key] = {};
            Object.keys(oldSendtask.settings[key]).forEach((key1) => {
              auxconfig[key][key1] = oldSendtask.settings[key][key1].default;
            });
          });
          auxbases.push(auxconfig);
          return { ...oldSendtask, bases: auxbases };
        });
      }
      if (meta.meth == 'del') {
        setSendTask((oldSendtask) => {
          let auxbases = JSON.parse(JSON.stringify(oldSendtask.bases));
          console.log(auxbases);
          auxbases.splice(meta.index, 1);
          return { ...oldSendtask, bases: auxbases };
        });
      }
    }
  };
  const setMarkersElements = (value) => {
    console.log(value);
    setMarkers({ ...markers, elements: value });
  };
  const SetMapMarkers = (value) => {
    setMarkers(value);
  };
  const setLocations = (value) => {
    setSendTask({ ...SendTask, loc: value });
  };
  const managepoints = (old, point, objetivo) => {
    if (objetivo == 'path-object') {
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
    if (objetivo == 'object') {
      old.push({ type: 'powerTower', name: 'Elements', linea: true, items: [point] });
    }
    if (objetivo == 'point') {
      old.push({ type: 'powerTower', name: 'Elements', linea: true, items: [point] });
    }

    return old;
  };
  const addLocations = (value) => {
    console.log(value);
    setSendTask((oldSendtask) => {
      let auxloc = JSON.parse(JSON.stringify(oldSendtask.loc));
      console.log(oldSendtask.objetivo.type);
      return { ...oldSendtask, loc: managepoints(auxloc, value, oldSendtask.objetivo.type) };
    });
  };
  const updateObjetive = (newObjetive) => {
    console.log('update objetivo');
    setSendTask((oldSendtask) => {
      let myTask = JSON.parse(JSON.stringify(oldSendtask));
      myTask.objetivo = newObjetive;
      if (newObjetive.type !== oldSendtask.objetivo.type) {
        myTask.loc = [];
      }
      return myTask;
    });
  };
  const toggleChecked = () => {
    setChecked((prev) => !prev);
  };

  useEffectAsync(async () => {
    console.log('change objetivo');
    if (auxobjetive !== SendTask.objetivo.id) {
      console.log('are different');
      setauxobjetive(SendTask.objetivo.id);
      const response = await fetch(`/api/planning/missionparam/${SendTask.objetivo.id}`);
      if (response.ok) {
        console.log('response ok');
        const paramsResponse = await response.json();
        console.log(paramsResponse);
        if (paramsResponse.hasOwnProperty('settings')) {
          if (SendTask.bases.length == 0) {
            console.log('settings base lenght  zero');
            let config = markers.bases.map(() => {
              let auxconfig = {};
              Object.keys(paramsResponse).forEach((key) => {
                auxconfig[key] = {};
                Object.keys(paramsResponse[key]).forEach((key1) => {
                  auxconfig[key][key1] = paramsResponse[key][key1].default;
                });
              });
              return auxconfig;
            });
            setSendTask((SendTaskold) => {
              const myTask = JSON.parse(JSON.stringify(SendTaskold));
              myTask.bases = config;
              myTask.settings = paramsResponse;
              return myTask;
            });
          } else {
            console.log('change basesss');
            let auxconfig = {};
            Object.keys(paramsResponse['settings']).forEach((key1) => {
              auxconfig[key1] = paramsResponse['settings'][key1].default;
            });
            console.log(auxconfig);
            // setSendTask({ ...SendTask, settings: paramsResponse });
            setSendTask((SendTaskold) => {
              const myTask = JSON.parse(JSON.stringify(SendTaskold));
              myTask.bases = myTask.bases.map((value) => ({ ...value, settings: auxconfig }));
              myTask.settings = paramsResponse;
              console.log(myTask);
              return myTask;
            });
          }
        }
      } else {
        throw Error(await response.text());
      }
    } else {
      console.log('are equal');
    }
  }, [SendTask.objetivo, auxobjetive]);

  useEffect(() => {
    tabValue == 2 && SendTask.objetivo.id != 3 ? SetSelectMarkers(true) : SetSelectMarkers(false);
    tabValue == 2 && SendTask.objetivo.id == 3 ? SetCreateMarkers(true) : SetCreateMarkers(false);
  }, [tabValue, SendTask.objetivo]);

  useEffect(() => {
    setMarkers(sessionmarkers);
  }, [sessionmarkers]);

  useEffect(() => {
    setSendTask(sessionplanning);
  }, [sessionplanning]);

  useEffect(() => {
    dispatch(sessionActions.updateMarker(markers));
  }, [markers]);

  useEffect(() => {
    dispatch(sessionActions.updatePlanning(SendTask));
  }, [SendTask]);

  useEffect(() => {
    const intervalPlanning = setInterval(() => {
      console.log('response Planning');
      SetRequestPlanning((old) => old + 1);
    }, 5000);

    const fetchData = async () => {
      const response = await fetch(`http://${myhostname}:8004/get_plan?IDs=${SendTask.id}`);
      if (response.ok) {
        const data = await response.json();
        console.log(data);
        if (Array.isArray(data.results) && data.results.length > 0) {
          if (data.results[0].hasOwnProperty('route')) {
            console.log('response to check planning');
            SetRequestPlanning(10);
            dispatch(missionActions.updateMission({ ...data.results[0], version: '3' }));
          }
        }
      } else {
        throw Error(await response.text());
      }
    };
    if (requestPlanning < 4) {
      fetchData();
    } else {
      clearInterval(intervalPlanning);
    }

    return () => clearInterval(intervalPlanning);
  }, [requestPlanning]);

  return (
    <div className={classes.root}>
      <MissionController>
        <RosControl notification={showToast}>
          <Navbar />
          <Menu />
          <div className={classes.mapContainer}>
            <MapView>
              {checked && <MapMissions />}
              <MapMarkersCreate
                markers={markers}
                selectMarkers={SendTask.loc}
                showTitles={showTitles}
                showLines={showLines}
                moveMarkers={moveMarkers}
                setMarkers={SetMapMarkers}
                SelectItems={SelectMarkers}
                CreateItems={CreateMarkers}
                setLocations={addLocations}
              />
              <MapDefaultCamera />
            </MapView>
            <MapScale />
          </div>

          <div className={classes.sidebarStyle}>
            <div className={classes.middleStyle}>
              <Paper square>
                <Toolbar className={classes.toolbar}>
                  <IconButton edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
                    <ArrowBackIcon />
                  </IconButton>
                  <Typography variant="h6" className={classes.title}>
                    Planning
                  </Typography>
                  <Typography>Show Mission</Typography>
                  <Switch
                    checked={checked}
                    onChange={toggleChecked}
                    name="checkedA"
                    inputProps={{ 'aria-label': 'secondary checkbox' }}
                  />
                  <IconButton
                    onClick={() => {
                      SavePlanning({
                        ...SendTask,
                        markersbase: markers.bases,
                        elements: markers.elements,
                      });
                    }}
                  >
                    <SaveAltIcon />
                  </IconButton>
                  <IconButton onClick={DeleteMission}>
                    <DeleteIcon />
                  </IconButton>
                  <UploadButtons
                    readFile={(e) => {
                      readFile(e);
                    }}
                    typefiles=".yaml, .plan, .waypoint, .kml"
                  />
                </Toolbar>
                <div className={classes.list}>
                  <TabContext value={tabValue}>
                    <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
                      <TabList onChange={TabHandleChange} aria-label="Planning tabs">
                        <Tab label="Elements" value="1" />
                        <Tab label="Planning" value="2" />
                        <Tab label="Settings" value="3" />
                      </TabList>
                    </Box>
                    <TabPanel value="1">
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
                            <ElementList markers={markers.elements} setMarkers={setMarkersElements} />
                          </AccordionDetails>
                        </Accordion>
                        <Box textAlign="center">
                          <Button
                            variant="contained"
                            size="large"
                            className={classes.panelButton}
                            onClick={SendPlanning}
                          >
                            Save Global Markers
                          </Button>
                        </Box>
                      </div>
                    </TabPanel>
                    <TabPanel value="2">
                      <Box className={classes.details}>
                        <TextField
                          required
                          fullWidth
                          label="id"
                          type="number"
                          variant="standard"
                          value={SendTask.id ? SendTask.id : 123}
                          onChange={(event) => setSendTask({ ...SendTask, id: event.target.value })}
                        />
                        <TextField
                          required
                          fullWidth
                          label="Name Mission"
                          variant="standard"
                          value={SendTask.name ? SendTask.name : ' '}
                          onChange={(event) => setSendTask({ ...SendTask, name: event.target.value })}
                        />
                        <SelectField
                          emptyValue={null}
                          fullWidth
                          label="objetive"
                          value={SendTask.objetivo.id ? SendTask.objetivo.id : 1}
                          endpoint="/api/planning/missionstype"
                          keyGetter={(it) => it.id}
                          titleGetter={(it) => it.name}
                          onChange={(e, items) => {
                            updateObjetive(items[e.target.value]);
                          }}
                          getItems={(it) => {
                            console.log(it);
                            setSendTask({ ...SendTask, objetivo: it });
                          }}
                        />

                        <Accordion>
                          <AccordionSummary expandIcon={<ExpandMore />}>
                            <Typography>Points to inspect </Typography>
                          </AccordionSummary>
                          <AccordionDetails className={classes.details}>
                            <div className={classes.details}>
                              {SendTask.objetivo.hasOwnProperty('description') ? (
                                <Typography>{SendTask.objetivo.description}</Typography>
                              ) : (
                                <Typography>select doing click in the map</Typography>
                              )}
                              <SelectList Data={SendTask.loc} setData={setLocations} />
                            </div>
                          </AccordionDetails>
                        </Accordion>
                        <Divider />
                        <Accordion>
                          <AccordionSummary expandIcon={<ExpandMore />}>
                            <Typography>Meteo</Typography>
                          </AccordionSummary>
                          <AccordionDetails className={classes.details}>
                            <TextField
                              required
                              fullWidth
                              label="wind speed"
                              type="number"
                              variant="standard"
                              value={12}
                            />
                            <TextField
                              required
                              fullWidth
                              label="Wind direction"
                              type="number"
                              variant="standard"
                              value={12}
                            />
                            <TextField
                              required
                              fullWidth
                              label="Temperature"
                              type="number"
                              variant="standard"
                              value={12}
                            />
                            <TextField
                              required
                              fullWidth
                              label="Humidity"
                              type="number"
                              variant="standard"
                              value={12}
                            />
                            <TextField
                              required
                              fullWidth
                              label="Pressure"
                              type="number"
                              variant="standard"
                              value={12}
                            />
                          </AccordionDetails>
                        </Accordion>
                      </Box>
                    </TabPanel>
                    <TabPanel value="3">
                      <div className={classes.details}>
                        <BaseSettings
                          data={SendTask.bases}
                          param={SendTask.settings}
                          setData={setBaseSettings}
                          goToBase={goToBase}
                        />

                        <Box textAlign="center">
                          <Button
                            variant="contained"
                            size="large"
                            className={classes.panelButton}
                            onClick={SendPlanning}
                          >
                            Planning
                          </Button>
                          <Button
                            variant="contained"
                            size="large"
                            className={classes.panelButton}
                            onClick={MissionTask}
                          >
                            Planinng with Global Setting
                          </Button>
                          <Button
                            variant="contained"
                            size="large"
                            className={classes.panelButton}
                            onClick={() =>
                              setDefaultPlanning({
                                ...SendTask,
                                markersbase: markers.bases,
                                elements: markers.elements,
                              })
                            }
                          >
                            Save Global Settings
                          </Button>
                        </Box>
                        <Typography>{notification}</Typography>
                      </div>
                    </TabPanel>
                  </TabContext>
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
        </RosControl>
      </MissionController>
    </div>
  );
};

export default PlanningPage;
