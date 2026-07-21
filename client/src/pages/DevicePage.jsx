import React, { useState } from 'react';
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
import ReplayIcon from '@mui/icons-material/Replay';
import PublishIcon from '@mui/icons-material/Publish';
import EditIcon from '@mui/icons-material/Edit';
import DeleteIcon from '@mui/icons-material/Delete';
import CameraAltIcon from '@mui/icons-material/CameraAlt';
import HomeIcon from '@mui/icons-material/Home';
import StopCircleIcon from '@mui/icons-material/StopCircle';

import { useNavigate, useParams } from 'react-router-dom';
import PositionValue from '../components/ui/PositionValue';
import usePersistedState from '../shared/usePersistedState';
import DroneSensorVisualizer from './DroneSensorVisualizer';
import useFilter from '../components/devices/useFilter';
import MainMap from '../map/MainMap';
import CameraDevice from '../components/camera/CameraDevice';
import CommandCard from '../components/commands/CommandCard';

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

const DEFAULT_SENSOR_DATA = { front: 1, back: 3, left: 5, right: 9, up: 2, down: 8 };

const DevicePage = () => {
  const { classes } = useStyles();
  const navigate = useNavigate();

  const { id } = useParams();
  const [value, setValue] = React.useState(0);

  const positions = useSelector((state) => state.session.positions);
  const devicelist = useSelector((state) => state.devices.items);
  const sessionmarkers = useSelector((state) => state.session.markers);
  const routes = useSelector((state) => state.mission.route);

  const item = id ? positions[id] : undefined;
  const thisDevice = id ? devicelist[id] : {};
  const markers = sessionmarkers;

  const [keyword] = useState('');
  const [filter] = usePersistedState('filter', {
    statuses: [],
    groups: [],
  });
  const currentSensorData = item?.attributes?.obstacle_info
    ? {
        front: item.attributes.obstacle_info[1],
        back: item.attributes.obstacle_info[3],
        left: item.attributes.obstacle_info[4],
        right: item.attributes.obstacle_info[2],
        up: item.attributes.obstacle_info[5],
        down: item.attributes.obstacle_info[0],
      }
    : DEFAULT_SENSOR_DATA;

  const [filterSort] = usePersistedState('filterSort', '');
  const [filterMap] = usePersistedState('filterMap', false);
  const [openSendCommand, setOpenSendCommand] = useState(false);
  const { filteredPositions } = useFilter(keyword, filter, filterSort, filterMap, positions);

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
            justifyContent: 'space-between',
            display: 'flex',
            flexDirection: 'column',
          }}
        >
          <div
            style={{
              height: '50vh',
            }}
            className={classes.content}
          >
            <MainMap
              filteredPositions={filteredPositions}
              markers={markers}
              routes={routes}
              selectedPosition={id}
              filteredMissiondeviceid={id}
            />
          </div>
          <div style={{ padding: '15px', margin: '5px' }}>
            {Object.keys(thisDevice).length > 0 && <CameraDevice deviceId={thisDevice.id} />}
          </div>
          <div style={{ padding: '15px', marginTop: 'auto' }}>
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
                      Object.getOwnPropertyNames(item).flatMap((property) =>
                        property === 'attributes'
                          ? []
                          : [
                              <TableRow key={property}>
                                <TableCell>{property}</TableCell>

                                <TableCell>
                                  <PositionValue position={item} property={property} />
                                </TableCell>
                              </TableRow>,
                            ],
                      )}
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
            {item?.attributes?.obstacle_info && (
              <Paper>
                <Typography align="center" variant="h5" component="div" style={{ padding: '15px' }}>
                  Avoidance sensor
                </Typography>
                <div style={{ height: 280 }}>
                  <DroneSensorVisualizer sensorData={currentSensorData} />
                </div>
              </Paper>
            )}
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
