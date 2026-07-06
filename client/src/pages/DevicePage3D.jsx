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
  BottomNavigation,
  BottomNavigationAction,
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';

import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import CameraAltIcon from '@mui/icons-material/CameraAlt';
import HomeIcon from '@mui/icons-material/Home';
import StopCircleIcon from '@mui/icons-material/StopCircle';

import { useNavigate, useParams } from 'react-router-dom';
import PositionValue from '../components/ui/PositionValue';
import usePersistedState from '../shared/usePersistedState';
import DroneSensorVisualizer from './DroneSensorVisualizer';
import useFilter from '../shared/useFilter';
import { CameraWebRTCV4 } from '../components/camera/CameraWebRTCV4';
import { CameraV1 } from '../components/camera/CameraV1';
import CommandCard from '../components/commands/CommandCard';
import Scene3DCanvas from '../scene3d/Scene3DCanvas';

const useStyles = makeStyles()((theme) => ({
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

const DevicePage3D = () => {
  const { classes } = useStyles();
  const navigate = useNavigate();

  const { id } = useParams();

  const [item, setItem] = useState();
  const [thisDevice, setthisdevice] = useState({});
  const positions = useSelector((state) => state.session.positions);
  const devicelist = useSelector((state) => state.devices.items);
  const sessionmarkers = useSelector((state) => state.session.markers);

  const [, setmarkers] = useState([]);

  const myhostname = `${window.location.hostname}`;
  const [, setFilteredPositions] = useState([]);
  const [, setFilteredDevices] = useState([]);
  const [keyword] = useState('');
  const [filter] = usePersistedState('filter', {
    statuses: [],
    groups: [],
  });
  const [currentSensorData, setCurrentSensorData] = useState({
    front: 1,
    back: 3,
    left: 5,
    right: 9,
    up: 2,
    down: 8,
  });

  const [filterSort] = usePersistedState('filterSort', '');
  const [filterMap] = usePersistedState('filterMap', false);
  const [openSendCommand, setOpenSendCommand] = useState(false);
  useFilter(
    keyword,
    filter,
    filterSort,
    filterMap,
    positions,
    setFilteredDevices,
    setFilteredPositions,
  );

  useEffect(() => {
    setmarkers(sessionmarkers);
  }, [sessionmarkers]);

  useEffect(() => {
    if (id) {
      setItem(positions[id]);
    }
  }, [id, positions]);
  useEffect(() => {
    if (item?.attributes?.obstacle_info) {
      setCurrentSensorData({
        front: item.attributes.obstacle_info[1],
        back: item.attributes.obstacle_info[3],
        left: item.attributes.obstacle_info[4],
        right: item.attributes.obstacle_info[2],
        up: item.attributes.obstacle_info[5],
        down: item.attributes.obstacle_info[0],
      });
    }
  }, [item]);

  useEffect(() => {
    if (id) {
      setthisdevice(devicelist[id]);
      console.log(devicelist[id]);
    }
  }, [id, devicelist]);

  return (
    <div className={classes.root}>
      <AppBar position="sticky" color="inherit">
        <Toolbar>
          <IconButton color="inherit" edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
            <ArrowBackIcon />
          </IconButton>
          <Typography variant="h6">{thisDevice.name}</Typography>
        </Toolbar>
      </AppBar>
      <div style={{ display: 'flex', flex: 1, overflow: 'hidden' }}>
        <div
          style={{
            flex: 1,
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            minHeight: 0,
          }}
        >
          <div
            style={{
              height: '50vh',
              flexShrink: 0,
            }}
            className={classes.content}
          >
            {Object.keys(thisDevice).length > 0 && (
              <RenderCamera device={thisDevice} myhostname={myhostname} />
            )}
          </div>
          <div
            style={{
              padding: '0px',
              margin: '5px',
              display: 'flex',
              gap: '16px',
              alignItems: 'stretch',
              flex: 1,
              overflow: 'hidden',
              minHeight: 0,
            }}
          >
            <Container
              maxWidth="false"
              style={{
                flex: 1,
                padding: 0,
                overflow: 'auto',
                maxHeight: '100%',
                alignSelf: 'flex-start',
              }}
            >
              <Paper>
                <Table aria-label="simple table" stickyHeader>
                  <TableHead>
                    <TableRow>
                      <TableCell sx={{ fontWeight: 'bold' }}>Attributes</TableCell>
                      <TableCell sx={{ fontWeight: 'bold' }}>Value</TableCell>
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
            {item?.attributes?.obstacle_info && (
              <Paper
                style={{
                  flex: 1,
                  minWidth: 0,
                  overflow: 'hidden',
                  display: 'flex',
                  flexDirection: 'column',
                }}
              >
                <DroneSensorVisualizer
                  sensorData={currentSensorData}
                  altitude={
                    item.attributes?.home
                      ? item.altitude - item.attributes.home[2]
                      : (item.altitude ?? 0)
                  }
                  altitudeASL={item.altitude ?? 0}
                />
              </Paper>
            )}
          </div>
          <div style={{ marginTop: 'auto' }}>
            <Paper>
              <BottomNavigation showLabels>
                <BottomNavigationAction label="Stop Mission" icon={<StopCircleIcon />} />
                <BottomNavigationAction label="Resume Mission" icon={<ArrowBackIcon />} />
                <BottomNavigationAction
                  label="Go to Home"
                  icon={<HomeIcon />}
                  onClick={() => setOpenSendCommand(true)}
                />
                <BottomNavigationAction label="Take Photo" icon={<CameraAltIcon />} />
              </BottomNavigation>
            </Paper>
          </div>
        </div>
        <div
          style={{
            flex: 1,
            display: 'flex',
            flexDirection: 'column',
            minHeight: 0,
          }}
        >
          <Scene3DCanvas style={{ flex: 1, minHeight: 0 }} />
        </div>
      </div>
      {openSendCommand && <CommandCard id={id} onClose={() => setOpenSendCommand(false)} />}
    </div>
  );
};

export default DevicePage3D;
