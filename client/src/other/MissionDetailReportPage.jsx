import React, { useState, useEffect } from 'react';
import { useSelector } from 'react-redux';
import { useNavigate, useParams } from 'react-router-dom';

import {
  Typography,
  Container,
  Paper,
  AppBar,
  Toolbar,
  IconButton,
  ImageList,
  ImageListItem,
  Card,
  CardMedia,
  CardHeader,
  Divider,
  CardContent,
  Grid,
  Chip,
  Tab,
  Box,
  Accordion,
  AccordionSummary,
  AccordionDetails,
} from '@mui/material';
import { TabPanel, TabList, TabContext } from '@mui/lab';
import CloseIcon from '@mui/icons-material/Close';
import ExpandMore from '@mui/icons-material/ExpandMore';

import makeStyles from '@mui/styles/makeStyles';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';

import { formatTime } from '../common/formatter';

import { useEffectAsync } from '../reactHelper';
import MapView from '../Mapview/MapView';
import MapMissions from '../Mapview/MapMissions';
import MapMarkers from '../Mapview/MapMarkers';
import RoutesList from '../components/RoutesList';
import SelectField from '../common/components/SelectField';
import SelectList from '../components/SelectList';
import BaseSettings from '../components/BaseSettings';

const useStyles = makeStyles((theme) => ({
  root: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
  },
  content: {
    overflow: 'auto',
    paddingTop: theme.spacing(2),
    paddingBottom: theme.spacing(2),
  },
  buttons: {
    marginTop: theme.spacing(2),
    marginBottom: theme.spacing(2),
    display: 'flex',
    justifyContent: 'space-evenly',
    '& > *': {
      flexBasis: '33%',
    },
  },
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
  card: {
    pointerEvents: 'auto',
  },
  media: {
    //height: theme.dimensions.popupImageHeight,
    width: theme.dimensions.popupMaxWidth,
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
    background: 'black',
  },
  media1: {
    //height: theme.dimensions.popupImageHeight
    width: '95vw',
    height: '90vh',
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
    background: 'black',
  },
  gruopBtn: {
    display: 'flex',
    right: '5px',
    height: '40px',
    position: 'absolute',
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: 'difference',
  },
  tittle: {
    display: 'block',
    width: 'calc( 100% - 60pt )',
    paddingLeft: '15pt',
    paddingTop: '10pt',
    paddingBottom: '10pt',
    textAlign: 'left',
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: theme.spacing(1, 1, 0, 2),
  },
  root_max: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 6,
    left: '50%',
    top: '8vh',
    transform: 'translateX(-50%)',
  },
}));
const ImageFull = ({ file, closecard }) => {
  const classes = useStyles();
  return (
    <div className={classes.root_max}>
      {file && (
        <Card elevation={3} className={classes.card}>
          <CardHeader
            action={
              <IconButton aria-label="close" onClick={() => closecard()}>
                <CloseIcon />
              </IconButton>
            }
            title={file.name}
            subheader="date:September 14, 2016"
          />
          <CardMedia component="img" alt={file.name} image={`/api/files/download/${file.route}${file.name}`} />
          <CardContent>
            <Typography variant="body2" color="text.secondary">
              Result:{JSON.stringify(file.attributes)}
            </Typography>
          </CardContent>
        </Card>
      )}
    </div>
  );
};
const ImageFull2 = ({ file, closecard }) => {
  const classes = useStyles();

  return (
    <div className={classes.root_max}>
      {file && (
        <Card elevation={3} className={classes.card}>
          <div className={classes.gruopBtn}>
            <IconButton size="small" onClick={closecard}>
              <CloseIcon fontSize="small" className={classes.mediaButton} />
            </IconButton>
          </div>
          <div className={classes.tittle}>{file.name}</div>
          <img
            src={`/api/files/download/${file.route}${file.name}`}
            alt={file.name}
            loading="lazy"
            className={classes.media1}
          />
        </Card>
      )}
    </div>
  );
};

const MissionDetailReportPage = () => {
  const classes = useStyles();
  const navigate = useNavigate();
  const { id } = useParams();

  const [missions, setMissions] = useState(null);
  const [dataMission, setDataMission] = useState(null);
  const [dataParam, setDataParam] = useState(null);
  const [missionMarkers, setMissionMarkers] = useState({ elements: [], bases: [] });
  const [routes, setRoutes] = useState(null);

  const [files, setFiles] = useState(null);
  const [routePath, setRoutePath] = useState(null);
  const [selectFile, setSelectFile] = useState(null);

  const devices = useSelector((state) => state.devices.items);

  const [tabValue, setTabValue] = useState('1');

  const TabHandleChange = (event, newTabValue) => {
    setTabValue(newTabValue);
  };
  const missionItems = 'id,initTime,endTime,status';
  const routeItems = 'id,initTime,endTime,status,deviceId,result';

  const FormatResult = ({ result }) => {
    if (result && result.hasOwnProperty('measures') && result.measures.length > 0) {
      return result.measures.map((item, itemIndex) => (
        <Typography key={`m-${item.name}${itemIndex}`}>{`${item.name}: ${item.value}`}</Typography>
      ));
    }
    return null;
  };

  const formatValue = (item, key) => {
    const value = item[key];
    if (value === null || value === undefined) {
      return '';
    }
    switch (key) {
      case 'deviceId':
        return devices[value].name;
      case 'uav': {
        const uavsName = value.map((uav) => devices[uav].name);
        return uavsName.join(', ');
      }
      case 'initTime':
        return formatTime(value, 'minutes');
      case 'endTime':
        return formatTime(value, 'minutes');

      case 'status': {
        let typeColor = 'primary';
        typeColor = value === 'done' ? 'success' : typeColor;
        typeColor = value === 'cancel' ? 'warning' : typeColor;
        typeColor = value === 'error' ? 'error' : typeColor;
        typeColor = value === 'init' ? 'info' : typeColor;
        return <Chip color={typeColor} label={value} />;
      }
      case 'result':
        return FormatResult({ result: value });
      default:
        return value;
    }
  };

  useEffect(() => {
    const myBases = [];
    let myElements = [];
    if (missions?.mission?.route) {
      setRoutePath(missions.mission.route);
    }
    if (missions?.task?.devices) {
      const data = [];
      Object.values(missions.task.devices).forEach((deviceValue) => {
        const myData = { devices: {} };
        myData.devices.name = deviceValue.id;
        myData.devices.category = deviceValue.category;
        if (deviceValue?.settings) {
          myData.settings = deviceValue.settings;
          if (deviceValue?.settings?.base) {
            myBases.push({ latitude: deviceValue.settings.base[0], longitude: deviceValue.settings.base[1] });
          }
        }
        data.push(myData);
      });
      setDataMission(data);
    }
    if (missions?.task?.locations) {
      myElements = missions.task.locations.map((item) => ({ ...item, type: 'locPoint' }));
    }
    console.log(myBases);
    console.log(myElements);
    setMissionMarkers({ bases: myBases, elements: myElements });
  }, [missions]);

  useEffectAsync(async () => {
    const response = await fetch(`/api/missions?id=${id}`);
    if (response.ok) {
      const myMissions = await response.json();
      setMissions(myMissions);
      console.log(myMissions);
    } else {
      throw Error(await response.text());
    }
    const response2 = await fetch(`/api/missions/routes?missionId=${id}`);
    if (response2.ok) {
      const myroutes = await response2.json();
      setRoutes(myroutes);
      console.log(myroutes);
    } else {
      throw Error(await response.text());
    }
    const response3 = await fetch(`/api/files/get?missionId=${id}`);
    if (response3.ok) {
      const myfiles = await response3.json();
      setFiles(myfiles);
      console.log(myfiles);
    } else {
      throw Error(await response.text());
    }
  }, []);

  useEffectAsync(async () => {
    if (missions && missions.hasOwnProperty('task') && missions.task && missions.task.hasOwnProperty('case')) {
      const response = await fetch(`/api/planning/missionparam/${missions.task.case}`);
      if (response.ok) {
        const myParamSettings = await response.json();
        console.log(myParamSettings);
        if (myParamSettings && myParamSettings.hasOwnProperty('settings')) {
          myParamSettings.devices.name = { name: 'Device', type: 'string', default: 'uav_0' };
          delete myParamSettings.devices.id;
          console.log(myParamSettings);
          setDataParam(myParamSettings);
        }
      } else {
        throw Error(await response.text());
      }
    }
  }, [missions]);

  return (
    <div className={classes.root}>
      <AppBar position="sticky" color="inherit">
        <Toolbar>
          <IconButton color="inherit" edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
            <ArrowBackIcon />
          </IconButton>
          <Typography variant="h6" component="div">
            {`Mission Reports- ${id}`}
          </Typography>
        </Toolbar>
      </AppBar>
      <div className={classes.content}>
        <Container maxWidth="xl">
          <Paper style={{ padding: 30 }}>
            {missions && (
              <div>
                <Typography variant="h4" gutterBottom>
                  Resultado de la Misión
                </Typography>
                <Grid container spacing={2}>
                  {missionItems
                    .split(',')
                    .filter((key) => missions.hasOwnProperty(key))
                    .map((key) => (
                      <Grid item xs={6} key={`ms${key}`}>
                        <Typography variant="subtitle1" style={{ fontWeight: 'bold' }}>
                          {key}
                        </Typography>
                        {key === 'status' ? (
                          formatValue(missions, key)
                        ) : (
                          <Typography variant="body1">{formatValue(missions, key)}</Typography>
                        )}
                      </Grid>
                    ))}
                  <Grid item xs={6} key={`msresult`}>
                    <Typography variant="subtitle1" style={{ fontWeight: 'bold' }}>
                      Results
                    </Typography>
                    {missions.results.map((item, itemIndex) => (
                      <FormatResult result={item} key={`msrs_${itemIndex}`} />
                    ))}
                  </Grid>
                </Grid>
              </div>
            )}
            {routes &&
              routes.map((route, routeIndex) => (
                <div key={`rt${routeIndex}`}>
                  <Divider style={{ margin: '40px 0' }} />
                  <Typography variant="h5" gutterBottom>
                    {`Ruta-${routeIndex}`}
                  </Typography>
                  <Grid container spacing={2}>
                    {routeItems
                      .split(',')
                      .filter((key) => route.hasOwnProperty(key))
                      .map((key) => (
                        <Grid item xs={6} key={`rt${routeIndex}_${key}`}>
                          <Typography variant="subtitle1" style={{ fontWeight: 'bold' }}>
                            {key}
                          </Typography>
                          {key === 'status' ? (
                            formatValue(missions, key)
                          ) : (
                            <Typography variant="body1">{formatValue(route, key)}</Typography>
                          )}
                        </Grid>
                      ))}
                  </Grid>
                  {files && files.find((item) => item && item.routeId == route.id) && (
                    <>
                      <Typography variant="h6" gutterBottom style={{ marginTop: '20px' }}>
                        Route files
                      </Typography>
                      <ImageList sx={{ width: '100%', height: 500 }} cols={3}>
                        {files
                          .filter((item) => item.routeId == route.id && item.name.endsWith('.jpg'))
                          .map((item) => (
                            <ImageListItem key={item.id}>
                              <img
                                src={`/api/files/download/${item.route}${item.name}`}
                                alt={item.name}
                                loading="lazy"
                                onClick={() => setSelectFile(item)}
                              />
                            </ImageListItem>
                          ))}
                      </ImageList>
                    </>
                  )}
                </div>
              ))}
            {dataMission && (
              <div>
                <Typography variant="h6" gutterBottom style={{ marginTop: '20px' }}>
                  Mapa de mission
                </Typography>
                <div style={{ width: '100%', height: '500px', position: 'relative' }}>
                  <MapView>
                    <MapMissions filtereddeviceid={-1} routes={routePath} />
                    <MapMarkers markers={missionMarkers} />
                  </MapView>
                  <Paper
                    square
                    elevation={3}
                    style={{
                      width: '500px',
                      height: '480px',
                      position: 'absolute',
                      top: '10px',
                      left: '10px',
                      flexDirection: 'column',
                      display: 'flex',
                      overflowY: 'auto',
                    }}
                  >
                    <TabContext value={tabValue}>
                      <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
                        <TabList onChange={TabHandleChange} aria-label="lab API tabs example">
                          <Tab label="mission" value="1" />
                          <Tab label="Planning" value="2" />
                        </TabList>
                      </Box>
                      <TabPanel value="1">
                        {routePath && (
                          <RoutesList
                            mission={missions.mission}
                            setmission={() => null}
                            setScrool={() => null}
                            NoEdit={true}
                          />
                        )}
                      </TabPanel>
                      <TabPanel value="2">
                        {missions.task && (
                          <>
                            <SelectField
                              emptyValue={null}
                              fullWidth
                              label="objetive"
                              value={missions.task.case}
                              endpoint="/api/planning/missionstype"
                              keyGetter={(it) => it.id}
                              titleGetter={(it) => it.name}
                            />
                            <Accordion>
                              <AccordionSummary expandIcon={<ExpandMore />}>
                                <Typography>Interest elements</Typography>
                              </AccordionSummary>
                              <AccordionDetails className={classes.details}>
                                {missions.task.locations && <SelectList Data={missions.task.locations} />}
                              </AccordionDetails>
                            </Accordion>
                            <Divider />
                            <Accordion>
                              <AccordionSummary expandIcon={<ExpandMore />}>
                                <Typography>Devices</Typography>
                              </AccordionSummary>
                              <AccordionDetails className={classes.details}>
                                {dataMission && dataParam && (
                                  <BaseSettings
                                    data={dataMission}
                                    param={dataParam}
                                    setData={() => null}
                                    goToBase={() => null}
                                  />
                                )}
                              </AccordionDetails>
                            </Accordion>
                          </>
                        )}
                      </TabPanel>
                    </TabContext>
                  </Paper>
                </div>
              </div>
            )}
          </Paper>
        </Container>
      </div>
      <ImageFull file={selectFile} closecard={() => setSelectFile(null)} />
    </div>
  );
};

export default MissionDetailReportPage;
