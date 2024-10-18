import React, { useState, useEffect } from 'react';
import { useSelector } from 'react-redux';
import { useEffectAsync } from '../reactHelper';

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
} from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
import makeStyles from '@mui/styles/makeStyles';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';

import MapView from '../Mapview/MapView';
import MapMissions from '../Mapview/MapMissions';
import MapMarkers from '../Mapview/MapMarkers';
import { useNavigate, useParams } from 'react-router-dom';
import { formatTime } from '../common/formatter';

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
  const [routes, setRoutes] = useState(null);
  const [files, setFiles] = useState(null);
  const [routePath, setRoutePath] = useState(null);
  const [selectFile, setSelectFile] = useState(null);
  const devices = useSelector((state) => state.devices.items);

  const missionItems = 'id,initTime,endTime,status';
  const routeItems = 'id,initTime,endTime,status,deviceId,result';

  const formatValue = (item, key) => {
    const value = item[key];
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
        typeColor = value === 'init' ? 'neutral' : typeColor;
        return <Chip color={typeColor} label={value} />;
      }
      default:
        return value;
    }
  };

  const FormatResult = ({ result }) => {
    if (result && result.hasOwnProperty('measures') && result.measures.length > 0) {
      return result.measures.map((item) => <Typography>{`${item.name}: ${item.value}`}</Typography>);
    }
    return null;
  };

  useEffect(() => {
    if (missions && missions.hasOwnProperty('mission') && missions.mission.hasOwnProperty('route')) {
      setRoutePath(missions.mission.route);
    }
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
                      <Grid item xs={6} key={key}>
                        <Typography variant="subtitle1" style={{ fontWeight: 'bold' }}>
                          {key}
                        </Typography>
                        <Typography variant="body1">{formatValue(missions, key)}</Typography>
                      </Grid>
                    ))}
                  <Grid item xs={6}>
                    <Typography variant="subtitle1" style={{ fontWeight: 'bold' }}>
                      Results
                    </Typography>
                    {missions.results.map((item) => (
                      <FormatResult result={item} />
                    ))}
                  </Grid>
                </Grid>
              </div>
            )}
            {routes &&
              routes.map((route, routeIndex) => (
                <div key={routeIndex}>
                  <Divider style={{ margin: '40px 0' }} />
                  <Typography variant="h5" gutterBottom>
                    {`Ruta-${routeIndex}`}
                  </Typography>
                  <Grid container spacing={2}>
                    {routeItems
                      .split(',')
                      .filter((key) => route.hasOwnProperty(key))
                      .map((key) => (
                        <Grid item xs={6} key={key}>
                          <Typography variant="subtitle1" style={{ fontWeight: 'bold' }}>
                            {key}
                          </Typography>
                          {key === 'result' ? (
                            <FormatResult result={route.result} />
                          ) : (
                            <Typography variant="body1">{formatValue(route, key)}</Typography>
                          )}
                        </Grid>
                      ))}
                  </Grid>
                  {files && files.find((item) => item && item.routeId == route.id) && (
                    <>
                      <Typography variant="h6" gutterBottom style={{ marginTop: '20px' }}>
                        Archivos de la Ruta
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
                  <Typography variant="h6" gutterBottom style={{ marginTop: '20px' }}>
                    Mapa de mission
                  </Typography>
                  <div style={{ width: '100%', height: '500px' }}>
                    <MapView>
                      <MapMissions filtereddeviceid={-1} routes={routePath} />
                      <MapMarkers markers={[]} />
                    </MapView>
                  </div>
                </div>
              ))}
          </Paper>
        </Container>
      </div>
      <ImageFull file={selectFile} closecard={() => setSelectFile(null)} />
    </div>
  );
};

export default MissionDetailReportPage;
