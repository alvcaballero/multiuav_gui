import React, { Fragment, useState } from 'react';
import Navbar from '../components/layout/Navbar';
import { Menu } from '../components/layout/Menu';
import MainToolbar from '../components/layout/MainToolbar';
import { makeStyles } from 'tss-react/mui';

import DeviceList from '../components/devices/DeviceList';
import { Paper, Grid, Box } from '@mui/material';
import CameraDevice from '../components/camera/CameraDevice';

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

  const { classes } = useStyles();
  return (
    <div className={classes.root}>
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
                    <CameraDevice deviceId={device.id} cameraIndex={cam_index} />
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
    </div>
  );
};

export default CameraPage;
