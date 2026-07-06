import React, { useState, useEffect, useCallback, useRef } from 'react';

import { useDispatch, useSelector } from 'react-redux';
import YAML from 'yaml';

import { Paper, Box, Tab } from '@mui/material';
import { TabPanel, TabList, TabContext } from '@mui/lab';

import { makeStyles } from 'tss-react/mui';

import { useNavigate } from 'react-router-dom';
import { missionActions, sessionActions, activeMissionsActions, planningToLegacy } from '../store';

import MapView, { map } from '../map/core/MapView';
import Navbar from '../components/layout/Navbar';
import { Menu } from '../components/layout/Menu';
import { MapMissions } from '../map/mission/MapMissions';
import { RosControl } from '../components/commands/RosControl';
import MissionElevation from '../components/mission/MissionElevation';
import MapMarkersCreate from '../map/draw/MapMarkersCreate';
import MapScale from '../map/controls/MapScale';
import MapDefaultCamera from '../map/controls/MapDefaultCamera';
import MapMissionHome from '../map/mission/MapMissionHome';
import { useAsyncTask, useCatch } from '../reactHelper';
import {
  manageLocationPoints,
  validateUniqueDevices,
  transformLocationsForAPI,
} from '../services/planningService';

import PlanningToolbar from '../components/planning/PlanningToolbar';
import ElementsTab from '../components/planning/ElementsTab';
import PlanningTab from '../components/planning/PlanningTab';
import SettingsTab from '../components/planning/SettingsTab';

// Enums para las tabs
const TABS = {
  ELEMENTS: '1',
  PLANNING: '2',
  SETTINGS: '3',
};

const useStyles = makeStyles()((theme) => ({
  root: {
    height: '100vh',
    margin: '0',
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
  content: {
    display: 'flex',
    gap: '10px',
    flexDirection: 'column',
    margin: '20px',
    overflow: 'hidden',
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
  const { classes } = useStyles();
  const navigate = useNavigate();
  const dispatch = useDispatch();

  const [showTitles, setShowTitles] = useState(true);
  const [showLines, setShowLines] = useState(false);
  const [moveMarkers, setMoveMarkers] = useState(false);
  const [requestPlanning, setRequestPlanning] = useState(100);
  const myhostname = `${window.location.hostname}`;

  const markers = useSelector((state) => state.session.markers);
  const SendTask = useSelector((state) => state.session.planning);
  const routeMission = useSelector((state) => state.mission.route);

  const sendTaskRef = useRef(SendTask);
  useEffect(() => {
    sendTaskRef.current = SendTask;
  }, [SendTask]);

  const [auxobjetive, setAuxobjetive] = useState(-1);
  const [tabValue, setTabValue] = useState(TABS.PLANNING);
  const [notification, setNotification] = useState('');
  const [checked, setChecked] = useState(true);

  // --- Helpers de planning ---

  const fetchDevices = useCallback(async () => {
    const response = await fetch('/api/devices');
    if (!response.ok) throw new Error(await response.text());
    return await response.json();
  }, []);

  const mapAssignmentToTaskDevice = useCallback((assignment, devices, markers) => {
    const device = devices.find((d) => d.id === Number(assignment.device.id));
    if (!device) return null;
    const base = markers.bases.find((b) => b.id === assignment.baseId);
    if (!base) return null;
    return {
      id: device.name,
      category: device.category,
      settings: { ...assignment.settings, base: Object.values(base), landing_mode: 2 },
    };
  }, []);

  const mapAssignmentsToDevices = useCallback(
    (assignments, devices, markers) =>
      assignments.flatMap((a) => {
        if (a.device.id === '') return [];
        const taskDevice = mapAssignmentToTaskDevice(a, devices, markers);
        return taskDevice ? [taskDevice] : [];
      }),
    [mapAssignmentToTaskDevice],
  );

  const buildTaskPayload = useCallback(
    (legacyPlanning, taskDevices) => ({
      id: legacyPlanning.id,
      name: legacyPlanning.name,
      case: legacyPlanning.objetivo.case,
      meteo: legacyPlanning.meteo,
      locations: transformLocationsForAPI(legacyPlanning.loc),
      devices: taskDevices,
    }),
    [],
  );

  // --- Acciones principales ---

  const SendPlanning = useCatch(async () => {
    setNotification('');
    const legacyPlanning = planningToLegacy(SendTask, markers);
    const assignments = SendTask.assignments || [];

    const validation = validateUniqueDevices(assignments);
    if (!validation.isValid) {
      setNotification(validation.errorMsg);
      return null;
    }

    if (legacyPlanning.loc.length === 0) {
      setNotification('No elements to inspection');
      return null;
    }

    const devices = await fetchDevices();
    const taskDevices = mapAssignmentsToDevices(assignments, devices, markers);
    const taskPayload = buildTaskPayload(legacyPlanning, taskDevices);

    const response = await fetch(`http://${myhostname}:8004/mission_request`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(taskPayload),
    });
    if (!response.ok) throw new Error(await response.text());

    setRequestPlanning(0);
    return true;
  });

  const MissionTask = useCatch(async () => {
    const myTask = {
      id: SendTask.id,
      name: SendTask.name,
      objetivo: SendTask.objetivo.id,
      meteo: SendTask.meteo,
      locations: SendTask.loc.map((group) => ({
        name: group.name,
        items: group.items.map(({ latitude, longitude }) => ({ latitude, longitude })),
      })),
    };

    const response = await fetch('/api/missions/sendTask', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(myTask),
    });
    if (!response.ok) throw new Error(await response.text());
  });

  const SavePlanning = useCallback((value) => {
    const blob = new Blob([YAML.stringify(value)], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.download = `${value.name}.yaml`;
    link.href = url;
    link.click();
  }, []);

  const setDefaultPlanning = useCatch(async (value) => {
    const response = await fetch('api/planning/setDefault', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(value),
    });
    if (!response.ok) throw new Error(await response.text());
  });

  // --- Handlers de Redux ---

  const setMarkersBase = useCallback(
    (value, meta = {}) => {
      dispatch(sessionActions.updateMarker({ ...markers, bases: value }));

      if (meta.meth === 'del') {
        const baseIdToRemove = value[meta.index]?.id;
        if (baseIdToRemove) {
          const newAssignments = (SendTask.assignments || []).filter(
            (a) => a.baseId !== baseIdToRemove,
          );
          dispatch(sessionActions.updatePlanning({ ...SendTask, assignments: newAssignments }));
        }
      }
    },
    [dispatch, markers, SendTask],
  );

  const setMarkersElements = useCallback(
    (value) => dispatch(sessionActions.updateMarker({ ...markers, elements: value })),
    [dispatch, markers],
  );

  const SetMapMarkers = useCallback(
    (value) => dispatch(sessionActions.updateMarker(value)),
    [dispatch],
  );

  const setLocations = useCallback(
    (value) => dispatch(sessionActions.updatePlanning({ ...SendTask, loc: value })),
    [dispatch, SendTask],
  );

  const addLocations = useCallback(
    (value) => {
      const newLoc = manageLocationPoints(
        structuredClone(SendTask.loc),
        value,
        SendTask.objetivo.type,
      );
      dispatch(sessionActions.updatePlanning({ ...SendTask, loc: newLoc }));
    },
    [dispatch, SendTask],
  );

  const setBaseSettings = useCallback(
    (assignments) => dispatch(sessionActions.updatePlanning({ ...SendTask, assignments })),
    [dispatch, SendTask],
  );

  const updateObjetive = useCallback(
    (newObjetive) => {
      const myTask = structuredClone(sendTaskRef.current);
      myTask.objetivo = newObjetive;
      if (newObjetive.type !== sendTaskRef.current.objetivo.type) myTask.loc = [];
      dispatch(sessionActions.updatePlanning(myTask));
    },
    [dispatch],
  );

  // --- Handlers de UI ---

  const readFile = useCallback(
    (e) => {
      const file = e.target.files[0];
      if (!file) return;
      const fileReader = new FileReader();
      fileReader.readAsText(file);
      fileReader.onload = () => {
        let myTask = YAML.parse(fileReader.result);
        dispatch(
          sessionActions.updateMarker({
            ...markers,
            bases: myTask.markersbase || markers.bases,
            elements: myTask.elements || markers.elements,
          }),
        );
        delete myTask.markersbase;
        delete myTask.elements;
        dispatch(sessionActions.updatePlanning(myTask));
      };
    },
    [dispatch, markers],
  );

  const goToBase = useCallback(
    (baseId) => {
      const base = markers.bases.find((b) => b.id === baseId);
      if (base)
        map.flyTo({ center: [base.longitude, base.latitude], zoom: Math.max(map.getZoom(), 16) });
    },
    [markers.bases],
  );

  const TabHandleChange = useCallback((_event, newTabValue) => {
    setTabValue(newTabValue);
    setShowTitles([TABS.ELEMENTS, TABS.PLANNING, TABS.SETTINGS].includes(newTabValue));
    setShowLines(newTabValue === TABS.ELEMENTS);
    setMoveMarkers(newTabValue === TABS.ELEMENTS);
  }, []);

  const handleNavigateBack = useCallback(() => navigate(-1), [navigate]);

  const handleSavePlanning = useCallback(
    () => SavePlanning({ ...SendTask, markersbase: markers.bases, elements: markers.elements }),
    [SavePlanning, SendTask, markers.bases, markers.elements],
  );

  const handleSaveGlobalMarkers = useCallback(
    () =>
      setDefaultPlanning({ ...SendTask, markersbase: markers.bases, elements: markers.elements }),
    [setDefaultPlanning, SendTask, markers.bases, markers.elements],
  );

  const handleUpdatePlanningId = useCallback(
    (event) => dispatch(sessionActions.updatePlanning({ ...SendTask, id: event.target.value })),
    [dispatch, SendTask],
  );

  const handleUpdatePlanningName = useCallback(
    (event) => dispatch(sessionActions.updatePlanning({ ...SendTask, name: event.target.value })),
    [dispatch, SendTask],
  );

  const handleUpdateObjective = useCallback(
    (e, items) => updateObjetive(items[e.target.value]),
    [updateObjetive],
  );

  const handleGetItems = useCallback(
    (it) => dispatch(sessionActions.updatePlanningObjective(it)),
    [dispatch],
  );

  const handleResetPolling = useCallback(() => setRequestPlanning(3), []);

  const handleDeleteMission = useCallback(() => {
    // future implementation
  }, []);

  // --- Effects ---

  useAsyncTask(async () => {
    const objetivoId = SendTask?.objetivo?.id;
    if (objetivoId === undefined || objetivoId === null) return;
    if (auxobjetive === objetivoId) return;

    setAuxobjetive(objetivoId);
    const response = await fetch(`/api/planning/missionparam/${objetivoId}`);
    if (!response.ok) throw new Error(await response.text());

    const paramsResponse = await response.json();
    if (!paramsResponse.hasOwnProperty('settings')) return;

    const defaultConfig = Object.fromEntries(
      Object.entries(paramsResponse.settings).map(([k, v]) => [k, v.default]),
    );

    const myTask = structuredClone(SendTask);
    myTask.settingsSchema = paramsResponse;
    myTask.defaultSettings = defaultConfig;
    if (!myTask.assignments) myTask.assignments = [];
    myTask.assignments = myTask.assignments.map((a) => ({
      ...a,
      settings: { ...defaultConfig, ...a.settings },
    }));
    dispatch(sessionActions.updatePlanning(myTask));
  }, [SendTask.objetivo, auxobjetive]);

  const isInPlanningTab = tabValue === TABS.PLANNING;
  const SelectMarkers = isInPlanningTab && SendTask.objetivo.id !== 3;
  const CreateMarkers = isInPlanningTab && SendTask.objetivo.id === 3;

  useEffect(() => {
    const MAX_RETRIES = 12;
    const POLLING_INTERVAL = 5000;
    const SUCCESS_CODE = 100;

    if (requestPlanning >= SUCCESS_CODE || requestPlanning >= MAX_RETRIES) return;

    const fetchData = async () => {
      try {
        const response = await fetch(`http://${myhostname}:8004/get_plan?IDs=${SendTask.id}`);
        if (!response.ok) throw new Error('Network response was not ok');

        const data = await response.json();
        const planResult = data.results?.[SendTask.id];
        if (planResult?.hasOwnProperty('route')) {
          setRequestPlanning(SUCCESS_CODE);
          dispatch(missionActions.updateMission({ ...planResult, version: '3' }));
          // New plan replaces the editor — drop any active selection.
          dispatch(activeMissionsActions.selectMission(null));
          return;
        }
      } catch (error) {
        console.error('Error fetching planning data:', error);
      } finally {
        setRequestPlanning((old) => old + 1);
      }
    };

    const intervalId = setInterval(fetchData, POLLING_INTERVAL);

    return () => clearInterval(intervalId);
  }, [requestPlanning, SendTask.id, dispatch, myhostname]);

  return (
    <div className={classes.root}>
      <RosControl notification={showToast}>
        <Navbar />
        <Menu />
        <div className={classes.mapContainer}>
          <MapView>
            {checked && <MapMissions routes={routeMission} />}
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
            <MapMissionHome />
          </MapView>
          <MapScale />
        </div>

        <div className={classes.sidebarStyle}>
          <div className={classes.middleStyle}>
            <Paper square>
              <PlanningToolbar
                onBack={handleNavigateBack}
                onSave={handleSavePlanning}
                onDelete={handleDeleteMission}
                onReadFile={readFile}
                showMission={checked}
                onToggleShowMission={() => setChecked((prev) => !prev)}
              />
              <div className={classes.content}>
                <TabContext value={tabValue}>
                  <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
                    <TabList onChange={TabHandleChange} aria-label="Planning tabs">
                      <Tab label="Elements" value={TABS.ELEMENTS} />
                      <Tab label="Planning" value={TABS.PLANNING} />
                      <Tab label="Settings" value={TABS.SETTINGS} />
                    </TabList>
                  </Box>
                  <TabPanel value={TABS.ELEMENTS} sx={{ padding: 0 }}>
                    <ElementsTab
                      markers={markers}
                      setMarkersBase={setMarkersBase}
                      setMarkersElements={setMarkersElements}
                      onSaveGlobalMarkers={handleSaveGlobalMarkers}
                    />
                  </TabPanel>
                  <TabPanel value={TABS.PLANNING} sx={{ padding: 0 }}>
                    <PlanningTab
                      sendTask={SendTask}
                      onUpdateId={handleUpdatePlanningId}
                      onUpdateName={handleUpdatePlanningName}
                      onUpdateObjective={handleUpdateObjective}
                      onGetItems={handleGetItems}
                      setLocations={setLocations}
                    />
                  </TabPanel>
                  <TabPanel value={TABS.SETTINGS} sx={{ padding: 0 }}>
                    <SettingsTab
                      sendTask={SendTask}
                      markers={markers}
                      notification={notification}
                      onSetBaseSettings={setBaseSettings}
                      onGoToBase={goToBase}
                      onSendPlanning={SendPlanning}
                      onResetPolling={handleResetPolling}
                      onMissionTask={MissionTask}
                      onSaveGlobalMarkers={handleSaveGlobalMarkers}
                    />
                  </TabPanel>
                </TabContext>
              </div>
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
      </RosControl>
    </div>
  );
};

export default PlanningPage;
