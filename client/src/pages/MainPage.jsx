import React, { useContext, useState, useCallback, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Paper } from '@mui/material';
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
import MainMap from '../map/MainMap';
import StatusCard from '../components/devices/StatusCard';
import CameraDevice from '../components/camera/CameraDevice';
import ChatDrawer from '../components/chat/ChatDrawer';
import { devicesActions } from '../store';

const useStyles = makeStyles()((theme) => ({
  root: {
    height: '100%',
    width: '100%',
    position: 'relative',
    overflow: 'hidden',
    margin: 0,
    padding: 0,
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
    overflow: 'hidden', // Ensure inner content doesn't force scroll of container
  },
  contentMap: {
    pointerEvents: 'auto',
    gridArea: '1 / 1',
  },
  contentList: {
    pointerEvents: 'auto',
    gridArea: '1 / 1',
    zIndex: 4,
    height: '100%', // Take full height of 'middle'
    overflowY: 'auto', // Allow scrolling inside the list if needed
  },
  sidebar: {
    pointerEvents: 'none',
    display: 'flex',
    flexDirection: 'column',
    position: 'absolute',
    left: 0,
    top: '88px',
    bottom: 0,
    width: '360px',
    margin: '0px',
    zIndex: 3,
  },
  map:{
    position: 'absolute',
    top: '88px',
    left: '360px',
    right: '0px',
    bottom: '0px', 
  }
}));

const MainPage = () => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const theme = useTheme();

  const devices = useSelector((state) => state.devices.items);
  const mission = useSelector((state) => state.mission);
  const llmEnabled = useSelector((state) => state.session.server?.llmEnabled ?? false);
  const positions = useSelector((state) => state.session.positions);

  const handleCommandMission = useCatch(() => commandMission(mission, devices));
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
  const [chatOpen, setChatOpen] = useState(false);

  const [confirmMission, setconfirmMission] = useState(false);
  const memoSetAddUAVOpen = useCallback(SetAddUAVOpen, []);
  const memoSetConfirmMission = useCallback(setconfirmMission, []);
  const memoSetChatOpen = useCallback(setChatOpen, []);

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
      <Navbar
        SetAddUAVOpen={memoSetAddUAVOpen}
        setconfirmMission={memoSetConfirmMission}
        setChatOpen={memoSetChatOpen}
      />
      <RosControl>
        <Menu SetAddUAVOpen={memoSetAddUAVOpen} />
      </RosControl>

      <SwipeConfirm
        enable={confirmMission}
        onClose={() => setconfirmMission(false)}
        onSucces={() => handleCommandMission()}
      />
      <div className={classes.map}>
        <MainMap
          filteredPositions={filteredPositions}
          markers={markers}
          routes={routes}
          selectedPosition={selectedPosition}
        />
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
      {llmEnabled && <ChatDrawer open={chatOpen} onClose={() => setChatOpen(false)} />}
      <CameraDevice deviceId={selectDeviceId} onClose={unselectDevice} />
      {AddUAVOpen && <Adduav SetAddUAVOpen={SetAddUAVOpen} />}
    </div>
  );
};

export default MainPage;
