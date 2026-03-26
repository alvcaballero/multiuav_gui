import React, { useContext, useState, useCallback, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Paper, Tab, Tabs } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import { useTheme } from '@mui/material/styles';
import Navbar from '../components/layout/Navbar';
import { Menu } from '../components/layout/Menu';
import Adduav from '../components/devices/Adduav';
import { RosControl, RosContext } from '../components/commands/RosControl';
import { commandMission } from '../shared/fetchs';
import { useCatch } from '../reactHelper';

import DeviceList from '../components/devices/DeviceList';
import SwipeConfirm from '../shared/components/SwipeConfirm';
import MainToolbar from '../components/layout/MainToolbar';
import StatusCard from '../components/devices/StatusCard';
import CameraDevice from '../components/camera/CameraDevice';

import SelectDevice3D from '../scene3d/scene/SelectDevice3D';

import { devicesActions } from '../store';

import MapView from '../scene3d/core/MapView';
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
    top: '88px',
    height: 'calc(100% - 88px)',
    width: '360px',
    margin: '0px',
    zIndex: 3,
  },
}));

const MainPage3D = () => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const theme = useTheme();

  const devices = useSelector((state) => state.devices.items);
  const mission = useSelector((state) => state.mission);
  const positions = useSelector((state) => state.session.positions);

  const handleCommandMission = useCatch(() => commandMission(mission, devices));
  const cameradata = useSelector((state) => state.session.camera);
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const sessionmarkers = useSelector((state) => state.session.markers);
  const routes = useSelector((state) => state.mission.route);

  const [selectDeviceId, setSelecDeviceId] = useState(null);

  const [filteredPositions, setFilteredPositions] = useState([]);
  const [markers, setmarkers] = useState([]);
  const selectedPosition = filteredPositions.find(
    (position) => selectedDeviceId && position.deviceId === selectedDeviceId
  );
  const [filteredDevices, setFilteredDevices] = useState([]);

  //const selectedImage = filteredImages.find((camera) => selectedDeviceId && camera.deviceId == selectedDeviceId);

  const [AddUAVOpen, SetAddUAVOpen] = useState(false);
  const [confirmMission, setconfirmMission] = useState(false);
  const memoSetAddUAVOpen = useCallback(SetAddUAVOpen, []);
  const memoSetConfirmMission = useCallback(setconfirmMission, []);

  useEffect(() => {
    console.log('MainPage mounted');
    return () => {
      console.log('MainPage unmounted');
    };
  }, []);
  useEffect(() => {
    setmarkers(sessionmarkers);
  }, [sessionmarkers]);
  useEffect(() => {
    console.log('devices updated');
    setFilteredDevices(Object.values(devices));
  }, [devices]);
  useEffect(() => {
    setFilteredPositions(Object.values(positions));
  }, [positions]);

  useEffect(() => {
    setSelecDeviceId(selectedDeviceId);
  }, [selectedDeviceId]);

  const unselectDevice = useCallback(() => {
    dispatch(devicesActions.selectId(null));
  }, [dispatch]);

  return (
    <div className={classes.root}>
      <RosControl>
        <Navbar SetAddUAVOpen={memoSetAddUAVOpen} setconfirmMission={memoSetConfirmMission} />
        <Menu SetAddUAVOpen={memoSetAddUAVOpen} />
      </RosControl>
      <div
        style={{
          position: 'absolute',
          top: '88px',
          right: '0px',
          width: 'calc(100% - 360px)',
          height: 'calc(100vh - 88px)',
        }}
      >
        <MapView>
          <R3FMission routes={routes} />
          <R3DMarkers elements={markers} />
          <R3FDevices />
          <SelectDevice3D />
        </MapView>
        <DownloadYamlButton />
      </div>

      <div className={classes.sidebar}>
        <Paper square elevation={3} className={classes.header}>
          <MainToolbar SetAddUAVOpen={memoSetAddUAVOpen} />
        </Paper>
        <div className={classes.middle}>
          <Paper square className={classes.contentList}>
            <DeviceList devices={filteredDevices} />
          </Paper>
        </div>
      </div>
      {selectDeviceId && (
        <StatusCard
          deviceId={selectDeviceId}
          position={selectedPosition}
          onClose={unselectDevice}
          desktopPadding={theme.dimensions.drawerWidthDesktop}
        />
      )}
      <SwipeConfirm
        enable={confirmMission}
        onClose={() => setconfirmMission(false)}
        onSucces={() => handleCommandMission()}
      />
      <CameraDevice deviceId={selectDeviceId} onClose={unselectDevice} />
      {AddUAVOpen && <Adduav SetAddUAVOpen={SetAddUAVOpen} />}
    </div>
  );
};

export default MainPage3D;
