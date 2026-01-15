import React, { useState, useEffect, useCallback } from 'react';

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
  Icon,
} from '@mui/material';
import ExpandMore from '@mui/icons-material/ExpandMore';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import { TabPanel, TabList, TabContext } from '@mui/lab';
import SaveAltIcon from '@mui/icons-material/SaveAlt';
import DeleteIcon from '@mui/icons-material/Delete';
import ReplayIcon from '@mui/icons-material/Replay';

import { makeStyles } from 'tss-react/mui';

import { useNavigate } from 'react-router-dom';
import { missionActions, sessionActions, sessionSelectors, planningToLegacy } from '../store'; // here update device action with position of uav for update in map

import MapView, { map } from '../Mapview/MapView';
import Navbar from '../components/Navbar';
import { Menu } from '../components/Menu';
import MapMissions from '../Mapview/MapMissions';
import SelectField from '../common/components/SelectField';
import BaseList from '../components/BaseList';
import BaseSettings from '../components/BaseSettings';
import { RosControl } from '../components/RosControl';
import MissionElevation from '../components/MissionElevation';
import MapMarkersCreate from '../Mapview/draw/MapMarkersCreate';
import MapScale from '../Mapview/MapScale';
import ElementList from '../components/ElementList';
import { useEffectAsync, useCatch } from '../reactHelper';
import SelectList from '../components/SelectList';
import MapDefaultCamera from '../Mapview/MapDefaultCamera';
import UploadButtons from '../components/uploadButton';
import { manageLocationPoints, validateUniqueDevices, transformLocationsForAPI } from '../services/planningService';

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
  content: {
    display: 'flex',
    gap: '10px',
    flexDirection: 'column',
    margin: '20px',
    overflow: 'hidden',
  },
  tabPanelContent: {
    maxHeight: 'calc(100vh - 240px)',
    overflowY: 'auto',
    paddingRight: '8px',
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
  const { classes } = useStyles();
  const navigate = useNavigate();
  const dispatch = useDispatch();

  const [showTitles, setShowTitles] = useState(true);
  const [showLines, setShowLines] = useState(false);
  const [moveMarkers, SetMoveMarkers] = useState(false);
  const [SelectMarkers, SetSelectMarkers] = useState(false);
  const [CreateMarkers, SetCreateMarkers] = useState(false);
  const [requestPlanning, SetRequestPlanning] = useState(100);
  const myhostname = `${window.location.hostname}`;

  // Usar Redux directamente como fuente única de verdad
  const markers = useSelector((state) => state.session.markers);
  const SendTask = useSelector((state) => state.session.planning);
  const routeMission = useSelector((state) => state.mission.route);

  // Estado local solo para UI (no duplica datos de negocio)
  const [auxobjetive, setauxobjetive] = useState(-1);
  const [tabValue, setTabValue] = useState(TABS.PLANNING);
  const [notification, setNotification] = useState('');
  const [checked, setChecked] = useState(true);

  // Función auxiliar: Validar que no haya dispositivos repetidos
  const validateAssignments = useCallback((assignments) => {
    const validation = validateUniqueDevices(assignments);

    console.log(
      'Device IDs:',
      assignments.map((a) => a.device.id).filter((id) => id !== '')
    );

    if (!validation.isValid) {
      console.error('❌', validation.errorMsg);
      setNotification(validation.errorMsg);
      return false;
    }

    console.log('✅ All devices are unique');
    return true;
  }, []);

  // Función auxiliar: Transformar locations a formato de API (usa el servicio)
  const transformLocations = useCallback((locations) => {
    return transformLocationsForAPI(locations);
  }, []);

  // Función auxiliar: Obtener dispositivos desde la API
  const fetchDevices = useCallback(async () => {
    const response = await fetch('/api/devices');
    if (!response.ok) {
      throw new Error(await response.text());
    }
    return await response.json();
  }, []);

  // Función auxiliar: Mapear una asignación a un dispositivo de tarea
  const mapAssignmentToTaskDevice = useCallback((assignment, devices, markers) => {
    const device = devices.find((d) => d.id === Number(assignment.device.id));
    if (!device) {
      console.warn(`Device ${assignment.device.id} not found`);
      return null;
    }

    const base = markers.bases.find((b) => b.id === assignment.baseId);
    if (!base) {
      console.warn(`Base ${assignment.baseId} not found`);
      return null;
    }

    return {
      id: device.name,
      category: device.category,
      settings: {
        ...assignment.settings,
        base: Object.values(base),
        landing_mode: 2,
      },
    };
  }, []);

  // Función auxiliar: Mapear asignaciones a dispositivos de tarea
  const mapAssignmentsToDevices = useCallback(
    (assignments, devices, markers) => {
      return assignments
        .filter((assignment) => assignment.device.id !== '')
        .map((assignment) => mapAssignmentToTaskDevice(assignment, devices, markers))
        .filter(Boolean);
    },
    [mapAssignmentToTaskDevice]
  );

  // Función auxiliar: Construir el objeto de tarea para enviar
  const buildTaskPayload = useCallback(
    (legacyPlanning, taskDevices) => {
      return {
        id: legacyPlanning.id,
        name: legacyPlanning.name,
        case: legacyPlanning.objetivo.case,
        meteo: legacyPlanning.meteo,
        locations: transformLocations(legacyPlanning.loc),
        devices: taskDevices,
      };
    },
    [transformLocations]
  );

  // Función auxiliar: Enviar la tarea al servidor de planificación
  const submitPlanningRequest = useCallback(
    async (taskPayload) => {
      const response = await fetch(`http://${myhostname}:8004/mission_request`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(taskPayload),
      });

      if (!response.ok) {
        throw new Error(await response.text());
      }

      const result = await response.json();
      console.log('Planning request submitted:', result);
      return result;
    },
    [myhostname]
  );

  // Función principal: Enviar planning
  const SendPlanning = useCatch(async () => {
    console.log('=== SendPlanning START ===');
    console.log('SendTask:', SendTask);
    console.log('markers:', markers);
    setNotification('');

    // Convertir planning a formato legacy para compatibilidad
    const legacyPlanning = planningToLegacy(SendTask, markers);
    console.log('legacyPlanning:', legacyPlanning);

    // Validar asignaciones
    const assignments = SendTask.assignments || [];
    console.log('assignments:', assignments);

    if (!validateAssignments(assignments)) {
      console.log('❌ Validation failed: assignments not valid');
      return null;
    }

    // Validar que haya elementos para inspeccionar
    console.log('legacyPlanning.loc.length:', legacyPlanning.loc.length);
    if (legacyPlanning.loc.length === 0) {
      console.log('❌ No elements to inspection');
      setNotification('No elements to inspection');
      return null;
    }

    // Obtener lista de dispositivos disponibles
    console.log('Fetching devices...');
    const devices = await fetchDevices();
    console.log('devices:', devices);

    // Mapear asignaciones a dispositivos de tarea
    const taskDevices = mapAssignmentsToDevices(assignments, devices, markers);
    console.log('taskDevices:', taskDevices);

    // Construir payload de la tarea
    const taskPayload = buildTaskPayload(legacyPlanning, taskDevices);
    console.log('Task payload:', taskPayload);

    // Enviar solicitud de planificación
    console.log(`Sending request to: http://${myhostname}:8004/mission_request`);
    await submitPlanningRequest(taskPayload);
    console.log('✅ Request sent successfully');

    // Iniciar polling para obtener resultados
    SetRequestPlanning(0);
    console.log('=== SendPlanning END ===');

    return true;
  });

  const DeleteMission = useCallback(() => {
    // future implementation
    console.log('uno');
  }, []);
  const SavePlanning = useCallback((value) => {
    let fileData = YAML.stringify(value);
    const blob = new Blob([fileData], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.download = `${value.name}.yaml`;
    link.href = url;
    link.click();
  }, []);
  const MissionTask = useCallback(async () => {
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
  }, [SendTask]);
  const goToBase = useCallback(
    (baseId) => {
      // Buscar la base por ID en lugar de índice
      const base = markers.bases.find((b) => b.id === baseId);
      if (base) {
        map.flyTo({
          center: [base.longitude, base.latitude],
          zoom: 16,
        });
      }
    },
    [markers.bases]
  );

  /*
   * https://www.youtube.com/watch?v=K3SshoCXC2g
   */
  const readFile = useCallback(
    (e) => {
      const file = e.target.files[0];
      if (!file) return;

      const fileReader = new FileReader();
      fileReader.readAsText(file);
      fileReader.onload = () => {
        let myTask = YAML.parse(fileReader.result);

        // Actualizar markers usando dispatch
        dispatch(
          sessionActions.updateMarker({
            ...markers,
            bases: myTask.markersbase || markers.bases,
            elements: myTask.elements || markers.elements,
          })
        );

        // Limpiar propiedades que ya se guardaron en markers
        myTask.hasOwnProperty('markersbase') ? delete myTask.markersbase : null;
        myTask.hasOwnProperty('elements') ? delete myTask.elements : null;

        // Actualizar planning usando dispatch
        dispatch(sessionActions.updatePlanning(myTask));
      };
      fileReader.onerror = () => {
        console.error('Error reading file:', fileReader.error);
      };
    },
    [dispatch, markers]
  );

  const setDefaultPlanning = useCallback(async (value) => {
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
  }, []);
  const setBaseSettings = useCallback(
    (assignments) => {
      dispatch(sessionActions.updatePlanning({ ...SendTask, assignments }));
    },
    [dispatch, SendTask]
  );
  const TabHandleChange = useCallback((_event, newTabValue) => {
    setTabValue(newTabValue);

    // Mostrar títulos en tabs de Elements, Planning y Settings
    const shouldShowTitles = [TABS.ELEMENTS, TABS.PLANNING, TABS.SETTINGS].includes(newTabValue);
    setShowTitles(shouldShowTitles);

    // Mostrar líneas solo en tab de Elements
    setShowLines(newTabValue === TABS.ELEMENTS);

    // Permitir mover marcadores solo en tab de Elements
    SetMoveMarkers(newTabValue === TABS.ELEMENTS);
  }, []);

  const setMarkersBase = useCallback(
    (value, meta = {}) => {
      dispatch(sessionActions.updateMarker({ ...markers, bases: value }));

      if (meta.hasOwnProperty('meth')) {
        if (meta.meth == 'add') {
          // Al agregar una nueva base, no necesitamos crear una asignación automática
          // Solo se creará cuando el usuario asigne un dispositivo desde BaseSettings
          console.log('Nueva base agregada, sin asignación inicial');
        }
        if (meta.meth == 'del') {
          // Al eliminar una base, también eliminar su asignación si existe
          const baseIdToRemove = value[meta.index]?.id;
          if (baseIdToRemove) {
            const newAssignments = (SendTask.assignments || []).filter(
              (assignment) => assignment.baseId !== baseIdToRemove
            );
            dispatch(sessionActions.updatePlanning({ ...SendTask, assignments: newAssignments }));
          }
        }
      }
    },
    [dispatch, markers, SendTask]
  );
  const setMarkersElements = useCallback(
    (value) => {
      //console.log(value);
      dispatch(sessionActions.updateMarker({ ...markers, elements: value }));
    },
    [dispatch, markers]
  );
  const SetMapMarkers = useCallback(
    (value) => {
      dispatch(sessionActions.updateMarker(value));
    },
    [dispatch]
  );
  const setLocations = useCallback(
    (value) => {
      dispatch(sessionActions.updatePlanning({ ...SendTask, loc: value }));
    },
    [dispatch, SendTask]
  );
  // Gestionar puntos de ubicación (usa el servicio de planning)
  const managepoints = useCallback((locations, point, objetivoType) => {
    return manageLocationPoints(locations, point, objetivoType);
  }, []);
  const addLocations = useCallback(
    (value) => {
      console.log(value);
      let auxloc = JSON.parse(JSON.stringify(SendTask.loc));
      console.log(SendTask.objetivo.type);
      const newLoc = managepoints(auxloc, value, SendTask.objetivo.type);
      dispatch(sessionActions.updatePlanning({ ...SendTask, loc: newLoc }));
    },
    [dispatch, SendTask, managepoints]
  );
  const updateObjetive = useCallback(
    (newObjetive) => {
      console.log('update objetivo');
      let myTask = JSON.parse(JSON.stringify(SendTask));
      myTask.objetivo = newObjetive;
      if (newObjetive.type !== SendTask.objetivo.type) {
        myTask.loc = [];
      }
      dispatch(sessionActions.updatePlanning(myTask));
    },
    [dispatch, SendTask]
  );
  const toggleChecked = useCallback(() => {
    setChecked((prev) => !prev);
  }, []);

  const handleNavigateBack = useCallback(() => {
    navigate(-1);
  }, [navigate]);

  const handleSavePlanning = useCallback(() => {
    SavePlanning({
      ...SendTask,
      markersbase: markers.bases,
      elements: markers.elements,
    });
  }, [SavePlanning, SendTask, markers.bases, markers.elements]);

  const handleReadFile = useCallback((e) => {
    readFile(e);
  }, []);

  const handleSaveGlobalMarkers = useCallback(() => {
    setDefaultPlanning({
      ...SendTask,
      markersbase: markers.bases,
      elements: markers.elements,
    });
  }, [setDefaultPlanning, SendTask, markers.bases, markers.elements]);

  const handleUpdatePlanningId = useCallback(
    (event) => {
      dispatch(sessionActions.updatePlanning({ ...SendTask, id: event.target.value }));
    },
    [dispatch, SendTask]
  );

  const handleUpdatePlanningName = useCallback(
    (event) => {
      dispatch(sessionActions.updatePlanning({ ...SendTask, name: event.target.value }));
    },
    [dispatch, SendTask]
  );

  const handleUpdateObjective = useCallback(
    (e, items) => {
      updateObjetive(items[e.target.value]);
    },
    [updateObjetive]
  );

  const handleGetItems = useCallback(
    (it) => {
      console.log(it);
      dispatch(sessionActions.updatePlanning({ ...SendTask, objetivo: it }));
    },
    [dispatch, SendTask]
  );

  const handleResetPolling = useCallback(() => {
    SetRequestPlanning(3);
  }, []);

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
          // Extraer configuración por defecto
          let defaultConfig = {};
          Object.keys(paramsResponse['settings']).forEach((key1) => {
            defaultConfig[key1] = paramsResponse['settings'][key1].default;
          });

          const myTask = JSON.parse(JSON.stringify(SendTask));

          // Actualizar el schema de settings
          myTask.settingsSchema = paramsResponse;

          // Actualizar los defaultSettings
          myTask.defaultSettings = defaultConfig;

          // Inicializar assignments si no existe
          if (!myTask.assignments) {
            myTask.assignments = [];
          }

          // Actualizar settings de todas las asignaciones existentes
          myTask.assignments = myTask.assignments.map((assignment) => ({
            ...assignment,
            settings: { ...defaultConfig, ...assignment.settings },
          }));

          console.log(myTask);
          dispatch(sessionActions.updatePlanning(myTask));
        }
      } else {
        throw Error(await response.text());
      }
    } else {
      console.log('are equal');
    }
  }, [SendTask.objetivo, auxobjetive]);

  useEffect(() => {
    // En tab Planning: permitir selección de marcadores (excepto para objetivo id=3)
    const isInPlanningTab = tabValue === TABS.PLANNING;
    SetSelectMarkers(isInPlanningTab && SendTask.objetivo.id !== 3);

    // En tab Planning con objetivo id=3: permitir creación de marcadores
    SetCreateMarkers(isInPlanningTab && SendTask.objetivo.id === 3);
  }, [tabValue, SendTask.objetivo]);

  useEffect(() => {
    // Configuración del polling
    const MAX_RETRIES = 12; // 12 intentos * 5 segundos = 60 segundos máximo
    const POLLING_INTERVAL = 5000; // 5 segundos
    const SUCCESS_CODE = 100; // Código para indicar que ya no se debe hacer polling

    // No hacer polling si no se ha iniciado (requestPlanning >= 100) o si ya se completó
    if (requestPlanning >= SUCCESS_CODE || requestPlanning >= MAX_RETRIES) {
      return;
    }

    const fetchData = async () => {
      try {
        const response = await fetch(`http://${myhostname}:8004/get_plan?IDs=${SendTask.id}`);

        if (!response.ok) {
          console.error(`Error fetching plan: ${response.status} ${response.statusText}`);
          throw new Error('Network response was not ok');
        }

        const data = await response.json();
        console.log('Planning response:', data);

        // Verificar si hay resultados válidos
        if (data.results && Object.keys(data.results).length > 0) {
          const planResult = data.results[SendTask.id];

          if (planResult && planResult.hasOwnProperty('route')) {
            console.log('Planning completed successfully');
            SetRequestPlanning(SUCCESS_CODE); // Detener el polling
            dispatch(missionActions.updateMission({ ...planResult, version: '3' }));
            return;
          }
        }
      } catch (error) {
        console.error('Error fetching planning data:', error);
      } finally {
        // Incrementar el contador solo si no se completó exitosamente
        SetRequestPlanning((old) => old + 1);
      }
    };

    // Ejecutar la primera vez inmediatamente
    fetchData();

    // Configurar el intervalo solo si no hemos alcanzado el límite
    const intervalId = setInterval(() => {
      SetRequestPlanning((old) => {
        if (old >= MAX_RETRIES - 1) {
          console.warn(`Max polling attempts (${MAX_RETRIES}) reached. Stopping polling.`);
          clearInterval(intervalId);
          return SUCCESS_CODE;
        }
        return old + 1;
      });
    }, POLLING_INTERVAL);

    // Cleanup: limpiar el intervalo cuando el componente se desmonte o cambie la dependencia
    return () => {
      clearInterval(intervalId);
    };
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
            </MapView>
            <MapScale />
          </div>

          <div className={classes.sidebarStyle}>
            <div className={classes.middleStyle}>
              <Paper square>
                <Toolbar className={classes.toolbar}>
                  <IconButton edge="start" sx={{ mr: 2 }} onClick={handleNavigateBack}>
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
                  <IconButton onClick={handleSavePlanning}>
                    <SaveAltIcon />
                  </IconButton>
                  <IconButton onClick={DeleteMission}>
                    <DeleteIcon />
                  </IconButton>
                  <UploadButtons readFile={handleReadFile} typefiles=".yaml, .plan, .waypoint, .kml" />
                </Toolbar>
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
                      <div className={`${classes.details} ${classes.tabPanelContent}`}>
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
                            onClick={handleSaveGlobalMarkers}
                          >
                            Save Global Markers
                          </Button>
                        </Box>
                      </div>
                    </TabPanel>
                    <TabPanel value={TABS.PLANNING} sx={{ padding: 0 }}>
                      <Box className={`${classes.details} ${classes.tabPanelContent}`}>
                        <TextField
                          required
                          fullWidth
                          label="id"
                          type="number"
                          variant="standard"
                          value={SendTask.id ? SendTask.id : 123}
                          onChange={handleUpdatePlanningId}
                        />
                        <TextField
                          required
                          fullWidth
                          label="Name Mission"
                          variant="standard"
                          value={SendTask.name ? SendTask.name : ' '}
                          onChange={handleUpdatePlanningName}
                        />
                        <SelectField
                          emptyValue={null}
                          fullWidth
                          label="objetive"
                          value={SendTask.objetivo.hasOwnProperty('id') ? SendTask.objetivo.id : 1}
                          endpoint="/api/planning/missionstype"
                          keyGetter={(it) => it.id}
                          titleGetter={(it) => it.name}
                          onChange={handleUpdateObjective}
                          getItems={handleGetItems}
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
                    <TabPanel value={TABS.SETTINGS} sx={{ padding: 0 }}>
                      <div className={`${classes.details} ${classes.tabPanelContent}`}>
                        <BaseSettings
                          data={SendTask.assignments || []}
                          markers={markers}
                          param={SendTask.settingsSchema}
                          defaultSettings={SendTask.defaultSettings}
                          setData={setBaseSettings}
                          goToBase={goToBase}
                        />

                        <Box textAlign="center">
                          <div className={classes.panelButton} style={{ marginLeft: 'auto', marginRight: 'auto' }}>
                            <Button variant="contained" sx={{ width: '80%' }} onClick={SendPlanning}>
                              Planning
                            </Button>
                            <Button
                              variant="contained"
                              color="secondary"
                              size="large"
                              sx={{ width: '20%' }}
                              endIcon={<ReplayIcon />}
                              onClick={handleResetPolling}
                            />
                          </div>

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
                            onClick={handleSaveGlobalMarkers}
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
    </div>
  );
};

export default PlanningPage;
