import React, { useState, useEffect } from 'react';
import { useSelector } from 'react-redux';

import {
  Typography,
  Container,
  Paper,
  AppBar,
  Toolbar,
  IconButton,
  Table,
  TableHead,
  TableRow,
  TableCell,
  TableBody,
  Box,
  BottomNavigation,
  BottomNavigationAction,
  Button,
} from '@mui/material';
import makeStyles from '@mui/styles/makeStyles';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import RestoreIcon from '@mui/icons-material/Restore';
import FavoriteIcon from '@mui/icons-material/Favorite';
import LocationOnIcon from '@mui/icons-material/LocationOn';
import { useNavigate, useParams } from 'react-router-dom';
import PositionValue from '../components/PositionValue';
import { useCatch } from '../reactHelper';
import SquareMove from './SquareMove';

import MapView from '../Mapview/MapView';
import MapPositions from '../Mapview/MapPositions';
import MapMissions from '../Mapview/MapMissions';
import MapMarkers from '../Mapview/MapMarkers';
import MapSelectedDevice from '../Mapview/MapSelectedDevice';

import { CameraWebRTCV4 } from '../components/CameraWebRTCV4';
import { CameraV1 } from '../components/CameraV1';

const useStyles = makeStyles((theme) => ({
  root: {
    margin: '0',
    height: '100vh',
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
}));

const RenderCamera = ({ device, myhostname }) => {
  const camera = device.camera && device.camera.length > 0 ? device.camera[0] : { type: '' };
  return (
    <>
      {camera.type === 'WebRTC' && (
        <CameraWebRTCV4
          deviceId={device.id}
          deviceIp={myhostname}
          devicename={device.name}
          camera_src={device.name + '_' + camera.source}
          onClose={() => {
            console.log('cerrar ');
          }}
        />
      )}
      {camera.type === 'WebRTC_env' && (
        <CameraWebRTCV4
          deviceId={device.id}
          deviceIp={device.ip}
          devicename={device.name}
          camera_src={camera.source}
          onClose={() => {
            console.log('cerrar ');
          }}
        />
      )}
      {camera.type === 'Websocket' && (
        <CameraV1 deviceId={device.id} datacamera={null} onClose={() => console.log('cerrar ')} />
      )}
    </>
  );
};

const DevicePage = () => {
  const classes = useStyles();
  const navigate = useNavigate();

  const { id } = useParams();
  const [value, setValue] = React.useState(0);

  const [item, setItem] = useState();
  const [thisDevice, setthisdevice] = useState({});
  const deviceposition = useSelector((state) => state.data.positions);
  const devicelist = useSelector((state) => state.devices.items);
  const [savedId, setSavedId] = useState(0);
  const limitCommands = 0;
  const [markers, setmarkers] = useState([]);
  const [filteredPositions, setFilteredPositions] = useState([]);
  const myhostname = `${window.location.hostname}`;

  const onMarkerClick = () => {};
  useEffect(() => {
    if (id) {
      setItem(deviceposition[id]);
    }
  }, [id, deviceposition]);

  useEffect(() => {
    if (id) {
      setthisdevice(devicelist[id]);
      console.log(devicelist[id]);
    }
  }, [id]);

  return (
    <div className={classes.root}>
      <AppBar position='sticky' color='inherit'>
        <Toolbar>
          <IconButton color='inherit' edge='start' sx={{ mr: 2 }} onClick={() => navigate(-1)}>
            <ArrowBackIcon />
          </IconButton>
          <Typography variant='h6'>{thisDevice.name}</Typography>
        </Toolbar>
      </AppBar>
      <div style={{ display: 'flex', flex: 1, overflow: 'hidden' }}>
        <div
          style={{
            flex: 1,
          }}
        >
          <div
            style={{
              height: '50vh',
            }}
          >
            <MapView>
              <MapMarkers markers={markers} />
              <MapMissions />
              <MapPositions
                positions={filteredPositions}
                onClick={onMarkerClick}
                selectedPosition={item}
                showStatus
              />
              <MapSelectedDevice />
            </MapView>
          </div>
          <div>
            {Object.keys(thisDevice).length > 0 && (
              <RenderCamera device={thisDevice} myhostname={myhostname} />
            )}
          </div>
        </div>
        <div
          style={{
            flex: 1,
          }}
        >
          <div style={{ height: '50vh' }} className={classes.content}>
            <Container maxWidth='sm'>
              <Paper>
                <Table aria-label='simple table'>
                  <TableHead>
                    <TableRow>
                      <TableCell>Attribute</TableCell>
                      <TableCell>value</TableCell>
                    </TableRow>
                  </TableHead>
                  <TableBody>
                    {item &&
                      Object.getOwnPropertyNames(item)
                        .filter((it) => it !== 'attributes')
                        .map((property) => (
                          <TableRow key={property}>
                            <TableCell>{property}</TableCell>

                            <TableCell>
                              <PositionValue position={item} property={property} />
                            </TableCell>
                          </TableRow>
                        ))}
                    {item &&
                      Object.getOwnPropertyNames(item.attributes).map((attribute) => (
                        <TableRow key={attribute}>
                          <TableCell>{attribute}</TableCell>
                          <TableCell>
                            <PositionValue position={item} attribute={attribute} />
                          </TableCell>
                        </TableRow>
                      ))}
                  </TableBody>
                </Table>
              </Paper>
            </Container>
          </div>
          <div style={{ padding: '20px', margin: '10px' }}>
            <Paper>
              <Typography align='center' variant='h5' component='div'>
                Avoidance sensor
              </Typography>
              {item && item.attributes && (
                <div style={{ display: 'flex' }}>
                  <SquareMove front_view={true} data={item.attributes.obstacle_info}></SquareMove>
                  <div style={{ width: '10px' }}></div>
                  <SquareMove front_view={false} data={item.attributes.obstacle_info}></SquareMove>
                </div>
              )}
            </Paper>
          </div>

          <Box style={{ Button: '0px' }}>
            <BottomNavigation
              showLabels
              value={value}
              onChange={(event, newValue) => {
                setValue(newValue);
              }}
            >
              <BottomNavigationAction label='Recents' icon={<RestoreIcon />} />
              <BottomNavigationAction label='Favorites' icon={<FavoriteIcon />} />
              <BottomNavigationAction label='Nearby' icon={<LocationOnIcon />} />
            </BottomNavigation>
          </Box>
        </div>
      </div>
    </div>
  );
};

export default DevicePage;
