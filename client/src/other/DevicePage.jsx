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
import makeStyles from '@mui/styles/makeStyles';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import ReplayIcon from '@mui/icons-material/Replay';
import PublishIcon from '@mui/icons-material/Publish';
import EditIcon from '@mui/icons-material/Edit';
import DeleteIcon from '@mui/icons-material/Delete';

import { useNavigate, useParams } from 'react-router-dom';
import PositionValue from '../components/PositionValue';
import usePersistedState from '../common/usePersistedState';
import SquareMove from './SquareMove';
import useFilter from '../common/useFilter';
import MainMap from '../Mapview/MainMap';
import { CameraWebRTCV4 } from '../components/CameraWebRTCV4';
import { CameraV1 } from '../components/CameraV1';
import SendCommand from '../components/SendCommand';
import CommandCard from '../components/CommandCard';

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
  const positions = useSelector((state) => state.session.positions);
  const devicelist = useSelector((state) => state.devices.items);
  const sessionmarkers = useSelector((state) => state.session.markers);
  const routes = useSelector((state) => state.mission.route);

  const [savedId, setSavedId] = useState(0);
  const limitCommands = 0;
  const [markers, setmarkers] = useState([]);

  const myhostname = `${window.location.hostname}`;
  const [filteredPositions, setFilteredPositions] = useState([]);
  const [filteredDevices, setFilteredDevices] = useState([]);
  const [keyword, setKeyword] = useState('');
  const [filter, setFilter] = usePersistedState('filter', {
    statuses: [],
    groups: [],
  });
  const [filterSort, setFilterSort] = usePersistedState('filterSort', '');
  const [filterMap, setFilterMap] = usePersistedState('filterMap', false);
  const [openSendCommand, setOpenSendCommand] = useState(false);
  useFilter(keyword, filter, filterSort, filterMap, positions, setFilteredDevices, setFilteredPositions);

  const onMarkerClick = () => {};
  useEffect(() => {
    setmarkers(sessionmarkers);
  }, [sessionmarkers]);

  useEffect(() => {
    if (id) {
      setItem(positions[id]);
    }
  }, [id, positions]);
  //  useEffect(() => {
  //    console.log(filteredPositions);
  //  }, [filteredPositions]);

  useEffect(() => {
    if (id) {
      setthisdevice(devicelist[id]);
      console.log(devicelist[id]);
    }
  }, [id]);

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
        <div style={{ flex: 1 }}>
          <div
            style={{
              height: '50vh',
            }}
          >
            <MainMap
              filteredPositions={filteredPositions}
              markers={markers}
              routes={routes}
              selectedPosition={id}
              filteredMissiondeviceid={id}
            />
          </div>
          <div>
            {Object.keys(thisDevice).length > 0 && <RenderCamera device={thisDevice} myhostname={myhostname} />}
          </div>
        </div>
        <div
          style={{
            flex: 1,
            justifyContent: 'space-between',
            display: 'flex',
            flexDirection: 'column',
          }}
        >
          <div style={{ height: '50vh' }} className={classes.content}>
            <Container maxWidth="false">
              <Paper>
                <Table aria-label="simple table">
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
          </div>
          <div style={{ padding: '15px', margin: '10px' }}>
            <Paper>
              <Typography align="center" variant="h5" component="div" style={{ padding: '15px' }}>
                Avoidance sensor
              </Typography>
              {item && item.attributes && (
                <div style={{ display: 'flex', flex: 1 }}>
                  <SquareMove front_view={true} data={item.attributes.obstacle_info}></SquareMove>

                  <SquareMove front_view={false} data={item.attributes.obstacle_info}></SquareMove>
                </div>
              )}
            </Paper>
          </div>

          <div style={{ padding: '15px', marginTop: 'auto' }}>
            <Paper>
              <BottomNavigation
                showLabels
                value={value}
                onChange={(event, newValue) => {
                  setValue(newValue);
                }}
              >
                <BottomNavigationAction label="Edit" icon={<EditIcon />} />
                <BottomNavigationAction label="Result" icon={<ReplayIcon />} />
                <BottomNavigationAction
                  label="Command"
                  icon={<PublishIcon />}
                  onClick={() => setOpenSendCommand(true)}
                />
                <BottomNavigationAction label="Delete" icon={<DeleteIcon />} />
              </BottomNavigation>
            </Paper>
          </div>
        </div>
      </div>
      {openSendCommand && <CommandCard id={id} onClose={() => setOpenSendCommand(false)} />}
    </div>
  );
};

export default DevicePage;
