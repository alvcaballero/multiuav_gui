import React, { useState, useCallback, useEffect, useMemo } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Paper } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import { useTheme } from '@mui/material/styles';
import Navbar from '../components/layout/Navbar';
import { Menu } from '../components/layout/Menu';
import Adduav from '../components/devices/Adduav';
import { RosControl } from '../components/commands/RosControl';
import { commandMission } from '../shared/fetchs';
import { useCatch } from '../reactHelper';

import DeviceList from '../components/devices/DeviceList';
import SwipeConfirm from '../shared/components/SwipeConfirm';
import MainToolbar from '../components/layout/MainToolbar';
import StatusCard from '../components/devices/StatusCard';
import CameraDevice from '../components/camera/CameraDevice';

import SelectDevice3D from '../scene3d/scene/SelectDevice3D';

import { devicesActions } from '../store';

import R3FCanvas from '../scene3d/core/R3FCanvas';
import R3FMission from '../scene3d/scene/R3FMission';
import R3DMarkers from '../scene3d/scene/R3DMarkers';
import R3FDevices from '../scene3d/scene/R3FDevices';
import DownloadYamlButton from '../scene3d/controls/DownloadYamlButton';

const useStyles = makeStyles()((theme) => ({
  root: {
    height: '100%',
  },
  header: {
    pointerEvents: 'auto',
    zIndex: 6,
  },
  footer: {
    pointerEvents: 'auto',
    zIndex: 5,
  },
  middle: {
    flex: 1,
    display: 'grid',
  },
  contentMap: {
    pointerEvents: 'auto',
    gridArea: '1 / 1',
  },
  contentList: {
    pointerEvents: 'auto',
    gridArea: '1 / 1',
    zIndex: 4,
  },
  sidebar: {
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
  canvas: {
    position: 'absolute',
    top: theme.dimensions.navbarHeight,
    right: '0px',
    width: `calc(100% - ${theme.dimensions.drawerWidthDesktop})`,
    height: `calc(100vh - ${theme.dimensions.navbarHeight})`,
  },
}));

const MainPage3D = () => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const theme = useTheme();

  const devicesMap = useSelector((state) => state.devices.items);
  const mission = useSelector((state) => state.mission);
  const positionsMap = useSelector((state) => state.session.positions);
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const sessionMarkers = useSelector((state) => state.session.markers);
  const routes = useSelector((state) => state.mission.route);

  const filteredDevices = useMemo(() => Object.values(devicesMap), [devicesMap]);
  const filteredPositions = useMemo(() => Object.values(positionsMap), [positionsMap]);
  const selectedPosition = useMemo(
    () => filteredPositions.find((p) => selectedDeviceId && p.deviceId === selectedDeviceId),
    [filteredPositions, selectedDeviceId]
  );

  const handleCommandMission = useCatch(() => commandMission(mission, devicesMap));

  const [addUAVOpen, setAddUAVOpen] = useState(false);
  const [confirmMission, setConfirmMission] = useState(false);

  const unselectDevice = useCallback(() => {
    dispatch(devicesActions.selectId(null));
  }, [dispatch]);

  return (
    <div className={classes.root}>
      <RosControl>
        <Navbar SetAddUAVOpen={setAddUAVOpen} setconfirmMission={setConfirmMission} />
        <Menu SetAddUAVOpen={setAddUAVOpen} />
      </RosControl>
      <div className={classes.canvas}>
        <R3FCanvas>
          <R3FMission routes={routes} />
          <R3DMarkers elements={sessionMarkers} />
          <R3FDevices />
          <SelectDevice3D />
        </R3FCanvas>
        <DownloadYamlButton />
      </div>

      <div className={classes.sidebar}>
        <Paper square elevation={3} className={classes.header}>
          <MainToolbar SetAddUAVOpen={setAddUAVOpen} />
        </Paper>
        <div className={classes.middle}>
          <Paper square className={classes.contentList}>
            <DeviceList devices={filteredDevices} />
          </Paper>
        </div>
      </div>
      {selectedDeviceId && (
        <StatusCard
          deviceId={selectedDeviceId}
          position={selectedPosition}
          onClose={unselectDevice}
          desktopPadding={theme.dimensions.drawerWidthDesktop}
        />
      )}
      <SwipeConfirm
        enable={confirmMission}
        onClose={() => setConfirmMission(false)}
        onSucces={() => handleCommandMission()}
      />
      <CameraDevice deviceId={selectedDeviceId} onClose={unselectDevice} />
      {addUAVOpen && <Adduav SetAddUAVOpen={setAddUAVOpen} />}
    </div>
  );
};

export default MainPage3D;
