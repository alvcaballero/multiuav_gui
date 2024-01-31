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
  Tab,
  Typography,
} from '@mui/material';
import ExpandMore from '@mui/icons-material/ExpandMore';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';

import TabContext from '@mui/lab/TabContext';
import TabList from '@mui/lab/TabList';
import TabPanel from '@mui/lab/TabPanel';

import makeStyles from '@mui/styles/makeStyles';
import { useNavigate } from 'react-router-dom';

import MapView from '../Mapview/MapView';
import { Navbar } from '../components/Navbar';
import { Menu } from '../components/Menu';
import MapMissions from '../Mapview/MapMissions';
import SelectField from '../common/components/SelectField';
import palette from '../common/palette';
import BaseList from '../components/BaseList';
import { RosControl } from '../components/RosControl';
import { MissionController } from '../components/MissionController';
import MissionElevation from '../components/MissionElevation';
import SaveFile from '../components/SaveFile';
import MapMarkersCreate from '../Mapview/draw/MapMarkersCreate';
import MapScale from '../Mapview/MapScale';
import ElementList from '../components/ElementList';

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
  const [Opensave, setOpenSave] = useState(false);

  const positions = useSelector((state) => state.data.positions);
  const sessionmarkers = useSelector((state) => state.session.markers);

  const [filteredPositions, setFilteredPositions] = useState([]);
  const [markers, setMarkers] = useState({ bases: [], elements: [] });
  const [SendTask, setSendTask] = useState({
    id: 1234,
    name: 'no mission',
    objetivo: 0,
    loc: [],
    meteo: [],
    setting: [],
  });
  const [value, setValue] = React.useState('1');

  const handleChange = (event, newValue) => {
    setValue(newValue);
  };

  const setMarkersBase = (value) => {
    setMarkers({ ...markers, bases: value });
  };
  const setMarkersElements = (value) => {
    setMarkers({ ...markers, elements: value });
  };

  useEffect(() => {
    setMarkers(sessionmarkers);
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
              float: 'right',
              width: 'calc(100% - 560px)',
              height: 'calc(70vh - 95px)',
              right: '0px',
              margin: 'auto',
            }}
          >
            <MapView>
              <MapMissions />
              <MapMarkersCreate markers={markers} showTitles={true} />
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
                    <TabContext value={value}>
                      <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
                        <TabList onChange={handleChange} aria-label='lab API tabs example'>
                          <Tab label='Elements' value='1' />
                          <Tab label='Planning' value='2' />
                          <Tab label='Settings' value='3' />
                        </TabList>
                      </Box>
                      <TabPanel value='1'>
                        <Accordion>
                          <AccordionSummary expandIcon={<ExpandMore />}>
                            <Typography>Base elements</Typography>
                          </AccordionSummary>
                          <AccordionDetails className={classes.details}>
                            <BaseList markers={markers.bases} setMarkers={setMarkersBase} />
                          </AccordionDetails>
                        </Accordion>
                        <Accordion>
                          <AccordionSummary expandIcon={<ExpandMore />}>
                            <Typography>Interest Elements</Typography>
                          </AccordionSummary>
                          <AccordionDetails className={classes.details}>
                            <ElementList
                              markers={markers.elements}
                              setMarkers={setMarkersElements}
                            ></ElementList>
                          </AccordionDetails>
                        </Accordion>
                      </TabPanel>
                      <TabPanel value='2'>
                        <Box>
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
                          />
                          <SelectField
                            fullWidth
                            emptyValue={null}
                            value={SendTask.objetivo}
                            onChange={(e) =>
                              setSendTask({
                                ...SendTask,
                                objetivo: e.target.value,
                              })
                            }
                            data={[
                              { id: 0, name: 'Electic line Inspection' },
                              { id: 1, name: 'Electric tower Inspection' },
                              { id: 2, name: 'Vegetation fire inspection' },
                            ]}
                            label={'objetive'}
                            style={{ display: 'inline', width: '200px' }}
                          />

                          <Accordion>
                            <AccordionSummary expandIcon={<ExpandMore />}>
                              <Typography>Points to inspect </Typography>
                            </AccordionSummary>
                            <AccordionDetails className={classes.details}></AccordionDetails>
                          </Accordion>
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
                        <Accordion>
                          <AccordionSummary expandIcon={<ExpandMore />}>
                            <Typography>Base 1 - UAV 2</Typography>
                          </AccordionSummary>
                          <AccordionDetails className={classes.details}>
                            <Typography>Meteo</Typography>
                          </AccordionDetails>
                        </Accordion>
                        <Accordion>
                          <AccordionSummary expandIcon={<ExpandMore />}>
                            <Typography>Base 2 - UAV 4</Typography>
                          </AccordionSummary>
                          <AccordionDetails className={classes.details}>
                            <Typography>Meteo</Typography>
                          </AccordionDetails>
                        </Accordion>
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
