import React, { Fragment, useState } from 'react';
import Navbar from '../components/layout/Navbar';
import { Menu } from '../components/layout/Menu';
import MainToolbar from '../components/layout/MainToolbar';
import { makeStyles } from 'tss-react/mui';

import { RosControl } from '../components/commands/RosControl';
import DeviceList from '../components/devices/DeviceList';
import { Paper, Grid, Box } from '@mui/material';
import { CameraWebRTCV4 } from '../components/camera/CameraWebRTCV4';
import { CameraV1 } from '../components/camera/CameraV1';

import { useSelector } from 'react-redux';
const useStyles = makeStyles()((theme) => ({
  root: {
    height: '100%',
  },
  sidebarStyle: {
    pointerEvents: 'none',
    display: 'flex',
    flexDirection: 'column',
    position: 'fixed',
    left: 0,
    top: theme.dimensions.navbarHeight,
    height: `calc(100% - ${theme.dimensions.navbarHeight})`,
    width: theme.dimensions.drawerWidthDesktop,
    margin: '0px',
    zIndex: 3,
  },
  middleStyle: {
    flex: 1,
    display: 'grid',
  },
  contentListStyle: {
    pointerEvents: 'auto',
    gridArea: '1 / 1',
    zIndex: 4,
  },
}));

const CameraPage = () => {
  const [addUAVOpen, setAddUAVOpen] = useState(false);
  void addUAVOpen;
  const devices = useSelector((state) => state.devices.items);
  let listdevices = Object.values(devices);
  const myhostname = `${window.location.hostname}`;

  const { classes } = useStyles();
  return (
    <div className={classes.root}>
      <RosControl>
        <Navbar />
        <Menu />
        <div
          style={{
            position: 'relative',
            width: '100%',
            height: `calc(100vh - 90px)`,
          }}
        >
          <Box
            component="div"
            sx={{ overflow: 'auto' }}
            style={{
              backgroundColor: '#000000',
              width: `calc(100vw - 360px)`,
              height: `calc(100vh - 90px`,
              padding: '20px',
              float: 'right',
            }}
          >
            <Grid container spacing={2} justifyContent="space-around">
              {Object.values(devices).map((device) => (
                <Fragment key={'dev' + device.id}>
                  {device.camera.map((camera, cam_index) => (
                    <Grid size={{ xs: 12, sm: 6 }} key={'card-' + device.id + '-' + cam_index}>
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
                        <CameraV1
                          deviceId={device.id}
                          datacamera={null}
                          onClose={() => console.log('cerrar ')}
                        />
                      )}
                    </Grid>
                  ))}
                </Fragment>
              ))}
            </Grid>
          </Box>
        </div>
        <div className={classes.sidebarStyle}>
          <div className={classes.middleStyle}>
            <Paper square className={classes.contentListStyle}>
              <MainToolbar SetAddUAVOpen={setAddUAVOpen} />
              <DeviceList devices={listdevices} />
            </Paper>
          </div>
        </div>
      </RosControl>
    </div>
  );
};

export default CameraPage;
