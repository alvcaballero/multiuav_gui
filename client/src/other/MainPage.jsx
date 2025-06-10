import React, { useContext, useState, useCallback, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Paper } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import { useTheme } from '@mui/material/styles';
import Navbar from '../components/Navbar';
import { Menu } from '../components/Menu';
import Toast from '../components/Toast';
import Adduav from '../components/Adduav';
import { RosControl, RosContext } from '../components/RosControl';
import DeviceList from '../components/DeviceList';
import SwipeConfirm from '../common/components/SwipeConfirm';
import MainToolbar from '../components/MainToolbar';
import MainMap from '../Mapview/MainMap';
import StatusCard from '../components/StatusCard';
import CameraDevice from '../components/CameraDevice';
import HelloWorld from './HelloWorld';

import { devicesActions } from '../store';

import CloseIcon from '@mui/icons-material/Close';
import { IconButton } from '@mui/material';

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
  middle: {
    flex: 1,
    display: 'grid',
  },
}));

const MainPage = () => {
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
  const [filteredImages, setFilteredImages] = useState([]);
  const [selectedDeviceIp, setselectedDeviceIp] = useState();
  const [selectedDeviceCam, setselectedDeviceCam] = useState();
  const [selectedDeviceCamsrc, setselectedDeviceCamsrc] = useState();
  const [selectedDeviceName, setselectedDeviceName] = useState();
  const selectedPosition = filteredPositions.find(
    (position) => selectedDeviceId && position.deviceId === selectedDeviceId
  );
  const [filteredDevices, setFilteredDevices] = useState([]);

  //const selectedImage = filteredImages.find((camera) => selectedDeviceId && camera.deviceId == selectedDeviceId);

  const [AddUAVOpen, SetAddUAVOpen] = useState(false);
  const [confirmMission, setconfirmMission] = useState(false);
  const memoSetAddUAVOpen = useCallback(SetAddUAVOpen, []);
  const memoSetConfirmMission = useCallback(setconfirmMission, []);

  const [list, setList] = useState([]);
  const showToast = (type, description) => {
    console.log(`${type} - ${description}`);
  };

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
    //console.log('Positions updated');
    setFilteredPositions(Object.values(positions));
  }, [positions]);
  useEffect(() => {
    setFilteredImages(Object.values(cameradata));
  }, [cameradata]);
  useEffect(() => {
    if (selectedDeviceId) {
      setselectedDeviceIp(devices[selectedDeviceId].ip);
      if (devices[selectedDeviceId].camera.length > 0) {
        const devicename = devices[selectedDeviceId].name;
        const camsrc = devices[selectedDeviceId].camera[0].source;
        const deviceCam = devices[selectedDeviceId].camera[0].type;
        const deviceCamSrc = selectedDeviceCam === 'WebRTC' ? `${devicename}_${camsrc}` : camsrc;
        setselectedDeviceCam(deviceCam);
        setselectedDeviceCamsrc(deviceCamSrc);
      } else {
        setselectedDeviceCam(null);
      }
      setselectedDeviceName(devices[selectedDeviceId].name);
    } else {
      setselectedDeviceIp(null);
      setselectedDeviceCam(null);
    }
    setSelecDeviceId(selectedDeviceId);
  }, [selectedDeviceId]);

  const unselectDevice = useCallback(() => {
    dispatch(devicesActions.selectId(null));
  }, [dispatch]);

  return (
    <div className={classes.root}>
      <RosControl notification={showToast}>
        <Navbar SetAddUAVOpen={memoSetAddUAVOpen} setconfirmMission={memoSetConfirmMission} />
        <Menu SetAddUAVOpen={memoSetAddUAVOpen} />

        <RosContext.Consumer>
          {({ commandMission }) => (
            <SwipeConfirm
              enable={confirmMission}
              onClose={() => setconfirmMission(false)}
              onSucces={() => commandMission()}
            />
          )}
        </RosContext.Consumer>
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
      {AddUAVOpen && <Adduav SetAddUAVOpen={SetAddUAVOpen} />}
      {selectDeviceId && (
        <StatusCard
          deviceId={selectDeviceId}
          position={selectedPosition}
          onClose={unselectDevice}
          desktopPadding={theme.dimensions.drawerWidthDesktop}
        />
      )}

      <CameraDevice deviceId={selectDeviceId} onClose={unselectDevice} />
      <Toast toastlist={list} position="buttom-right" setList={setList} />
    </div>
  );
};

export default MainPage;
