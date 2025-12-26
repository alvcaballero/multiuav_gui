import React, { useContext, useState, useCallback, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Paper, Tab, Tabs } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import { useTheme } from '@mui/material/styles';
import Navbar from '../components/Navbar';
import { Menu } from '../components/Menu';
import Adduav from '../components/Adduav';
import { RosControl, RosContext } from '../components/RosControl';
import { commandMission } from '../common/fetchs';

import DeviceList from '../components/DeviceList';
import SwipeConfirm from '../common/components/SwipeConfirm';
import MainToolbar from '../components/MainToolbar';
import StatusCard from '../components/StatusCard';
import CameraDevice from '../components/CameraDevice';

import SelectDevice3D from '../ThreeD/SelectDevice3D';

import { devicesActions } from '../store';

import MapView from '../ThreeD/MapView';
import R3FMission from '../ThreeD/R3FMission';
import R3DMarkers from '../ThreeD/R3DMarkers';
import R3FDevices from '../ThreeD/R3FDevices';
import DownloadYamlButton from '../ThreeD/DownloadYamlButton';

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
  const positions = useSelector((state) => state.session.positions);
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
        onSucces={() => commandMission()}
      />
      <CameraDevice deviceId={selectDeviceId} onClose={unselectDevice} />
      {AddUAVOpen && <Adduav SetAddUAVOpen={SetAddUAVOpen} />}
    </div>
  );
};

export default MainPage3D;
