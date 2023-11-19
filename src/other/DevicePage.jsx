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
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Button,
} from '@mui/material';
import makeStyles from '@mui/styles/makeStyles';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import { useNavigate, useParams } from 'react-router-dom';
import { prefixString } from '../common/stringUtils';
import PositionValue from '../components/PositionValue';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import SelectField from '../common/components/SelectField';
import BaseCommandView from '../common/components/BaseCommandView';
import { useCatch } from '../reactHelper';
import SquareMove from './SquareMove';

import MapView from '../Mapview/MapView';
import MapPositions from '../Mapview/MapPositions';
import MapMissions from '../Mapview/MapMissions';
import MapMarkers from '../Mapview/MapMarkers';
import MapSelectedDevice from '../Mapview/MapSelectedDevice';

import { CameraWebRTCV4 } from '../components/CameraWebRTCV4';
import { PortableWifiOff } from '@mui/icons-material';


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
}));

const RenderCamera =({device,myhostname})=>{
  if (device.hasOwnProperty('camera')){
    if(device.camera[0].type === 'WebRTC'){
      return(
        <CameraWebRTCV4
          deviceId={device.id}
          deviceIp={myhostname}
          devicename={device.name}
          camera_src={device.name + '_' + device.camera[0].source}
          onClose={() => {
            console.log('cerrar ');
          }}
        />)
    }
    else {
      return(
        <CameraWebRTCV4
          deviceId={device.id}
          deviceIp={device.ip}
          devicename={device.name}
          camera_src={device.camera[0].source}
          onClose={() => {
            console.log('cerrar ');
          }}
        />
      )
    }
  }else{
    return(null)
  }
}

const DevicePage = () => {
  const classes = useStyles();
  const navigate = useNavigate();

  const { id } = useParams();

  const [item, setItem] = useState();
  const [itemc, setItemc] = useState({});
  const [thisDevice, setthisdevice] = useState({});
  const deviceposition = useSelector((state) => state.data.positions);
  const devicelist = useSelector((state) => state.devices.items);
  const [savedId, setSavedId] = useState(0);
  const limitCommands = 0;
  const [markers, setmarkers] = useState([]);
  const [filteredPositions, setFilteredPositions] = useState([]);
  const myhostname = `${window.location.hostname}`;

  const onMarkerClick = ()=>{};
  useEffect(() => {
    if (id) {
      setItem(deviceposition[id]);
    }
  }, [id, deviceposition]);

  useEffect(() => {
    if (id) {
      setthisdevice(devicelist[id])
    }
  }, [id]);

  const deviceName = useSelector((state) => {
    if (item) {
      const device = state.devices.items[item.deviceId];
      if (device) {
        return device.name;
      }
    }
    return null;
  });

  const handleSend = useCatch(async () => {
    let command;
    if (savedId) {
      const response = await fetch(`/api/commands/${savedId}`);
      if (response.ok) {
        command = await response.json();
      } else {
        throw Error(await response.text());
      }
    } else {
      command = itemc;
    }

    command.deviceId = parseInt(id, 10);

    const response = await fetch('/api/commands/send', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(command),
    });

    if (response.ok) {
      navigate(-1);
    } else {
      throw Error(await response.text());
    }
  });

  const validate = () => savedId || (itemc && itemc.type);

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
      <div>
        <div
          style={{
            position: 'relative',
            display: 'inline-block',
            width: '49vw',
            height: `50vh`,
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
      <div className={classes.content}
                  style={{
                    display: 'inline-block',
                    position: 'relative',
                    width: '45vw',
                    height: `50vh`,
                    top: 0,
                    right:0
                  }}
      >
        <Container maxWidth='sm'>
          <Paper>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>stateName</TableCell>
                  <TableCell>stateValue</TableCell>
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
      <div

            style={{
              display: 'inline-block',
              position: 'relative',
              width: '47vw',
              height: `40vh`,
            }}
>
      <RenderCamera device={thisDevice} myhostname={myhostname}/>

      </div>
      <div
                  style={{
                    display: 'inline-block',
                    position: 'relative',
                    width: '50vw',
                    height: `40vh`,
                  }}
      >
        {item && item.attributes && (
          <div style={{display: 'flex'}}>
            <SquareMove front_view={true} data={item.attributes.obstacle_info}></SquareMove>
            <div style={{width:'10px'}}></div>
            <SquareMove front_view={false} data={item.attributes.obstacle_info}></SquareMove>
          </div>
        )}

        <div>
                <Container maxWidth='xs' className={classes.container}>
                  <Accordion defaultExpanded>
                    <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                      <Typography variant='subtitle1'>{'Command'}</Typography>
                    </AccordionSummary>
                    <AccordionDetails className={classes.details}>
                      {!limitCommands && !savedId && (
                        <BaseCommandView deviceId={id} item={itemc} setItem={setItemc} />
                      )}
                    </AccordionDetails>
                  </Accordion>
                  <div className={classes.buttons}>
                    <Button type='button' color='primary' variant='outlined' onClick={() => navigate(-1)}>
                      Cancel
                    </Button>
                    <Button
                      type='button'
                      color='primary'
                      variant='contained'
                      onClick={handleSend}
                      disabled={!validate()}
                    >
                      Send
                    </Button>
                  </div>
                </Container>
        </div>
      </div>

      </div>
    </div>
  );
};

export default DevicePage;
