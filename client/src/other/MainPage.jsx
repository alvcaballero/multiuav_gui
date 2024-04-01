import React, { useContext, useState, useCallback, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Paper } from '@mui/material';
import { useTheme } from '@mui/material/styles';
import makeStyles from '@mui/styles/makeStyles';
import Navbar from '../components/Navbar';
import { Menu } from '../components/Menu';
import Toast from '../components/Toast';
import { Adduav } from '../components/Adduav';
import { Camera } from '../components/Camera';
import { RosControl, RosContext } from '../components/RosControl';
import DeviceList from '../components/DeviceList';
import StatusCard from '../components/StatusCard';
import SwipeConfirm from '../common/components/SwipeConfirm';
import MainToolbar from '../components/MainToolbar';
import { CameraWebRTCV3 } from '../components/CameraWebRTCV3';
import MainMap from '../Mapview/MainMap';

import { devicesActions } from '../store';

const useStyles = makeStyles((theme) => ({
  root: {
    height: '100%',
  },
  sidebar: {
    pointerEvents: 'none',
    display: 'flex',
    flexDirection: 'column',
    [theme.breakpoints.up('md')]: {
      position: 'fixed',
      left: 0,
      top: 0,
      height: `calc(100% - ${theme.spacing(3)})`,
      width: theme.dimensions.drawerWidthDesktop,
      margin: theme.spacing(1.5),
      zIndex: 3,
    },
    [theme.breakpoints.down('md')]: {
      height: '100%',
      width: '100%',
    },
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
  sidebarStyle: {
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
  middleStyle: {
    flex: 1,
    display: 'grid',
  },
}));

const MainPage = () => {
  const classes = useStyles();
  const dispatch = useDispatch();
  const theme = useTheme();

  const devices = useSelector((state) => state.devices.items);
  const positions = useSelector((state) => state.data.positions);
  const cameradata = useSelector((state) => state.data.camera);
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const sessionmarkers = useSelector((state) => state.session.markers);
  const [filteredPositions, setFilteredPositions] = useState([]);
  const [markers, setmarkers] = useState([]);
  const [filteredImages, setFilteredImages] = useState([]);
  const [selectedDeviceIp, setselectedDeviceIp] = useState();
  const [selectedDeviceCam, setselectedDeviceCam] = useState();
  const [selectedDeviceCamsrc, setselectedDeviceCamsrc] = useState();
  const [selectedDeviceName, setlectedDeviceName] = useState();
  const myhostname = `${window.location.hostname}`;
  const selectedPosition = filteredPositions.find(
    (position) => selectedDeviceId && position.deviceId === selectedDeviceId
  );
  const selectedImage = filteredImages.find(
    (camera) => selectedDeviceId && camera.deviceId == selectedDeviceId
  );

  let listdevices = Object.values(devices);
  const rosContex = useContext(RosContext);

  const [AddUAVOpen, SetAddUAVOpen] = useState(false);

  const [list, setList] = useState([]);
  let toastProperties = null;
  const showToast = (type, description) => {
    console.log(`${type} - ${description}`);
  };

  const onMarkerClick = useCallback(
    (_, deviceId) => {
      dispatch(devicesActions.selectId(deviceId));
    },
    [dispatch]
  );
  useEffect(() => {
    setmarkers(sessionmarkers);
  }, [sessionmarkers]);
  useEffect(() => {
    setFilteredPositions(Object.values(positions));
  }, [positions]);
  useEffect(() => {
    setFilteredImages(Object.values(cameradata));
  }, [cameradata]);
  useEffect(() => {
    if (selectedDeviceId) {
      setselectedDeviceIp(devices[selectedDeviceId].ip);
      if (devices[selectedDeviceId].camera.length > 0) {
        setselectedDeviceCam(devices[selectedDeviceId].camera[0].type);
        setselectedDeviceCamsrc(devices[selectedDeviceId].camera[0].source);
      } else {
        setselectedDeviceCam(null);
      }
      setlectedDeviceName(devices[selectedDeviceId].name);
    } else {
      setselectedDeviceIp(null);
      setselectedDeviceCam(null);
    }
  }, [selectedDeviceId]);

  return (
    <div className={classes.root}>
      <RosControl notification={showToast}>
        <Navbar SetAddUAVOpen={SetAddUAVOpen} />
        <Menu SetAddUAVOpen={SetAddUAVOpen} />
        <div
          style={{
            position: 'relative',
            float: 'right',
            width: 'calc(100% - 360px)',
            height: 'calc(100vh - 88px)',
          }}
        >
          <MainMap
            filteredPositions={filteredPositions}
            markers={markers}
            selectedPosition={selectedPosition}
          />
        </div>

        <div className={classes.sidebarStyle}>
          <Paper square elevation={3} className={classes.header}>
            <MainToolbar SetAddUAVOpen={SetAddUAVOpen} />
          </Paper>
          <div className={classes.middleStyle}>
            <Paper square className={classes.contentList}>
              <DeviceList devices={listdevices} />
            </Paper>
          </div>
        </div>
        {AddUAVOpen && <Adduav SetAddUAVOpen={SetAddUAVOpen} />}
        <StatusCard
          deviceId={selectedDeviceId}
          position={selectedPosition}
          onClose={() => dispatch(devicesActions.selectId(null))}
          desktopPadding={theme.dimensions.drawerWidthDesktop}
        />
        <RosContext.Consumer>
          {({ confirmMission, setconfirmMission, commandMission }) => (
            <SwipeConfirm
              enable={confirmMission}
              onClose={() => setconfirmMission(false)}
              onSucces={() => commandMission()}
            />
          )}
        </RosContext.Consumer>

        {selectedDeviceCam === 'WebRTC' && (
          <CameraWebRTCV3
            deviceId={selectedDeviceId}
            deviceIp={myhostname}
            camera_src={selectedDeviceName + '_' + selectedDeviceCamsrc}
            onClose={() => dispatch(devicesActions.selectId(null))}
          />
        )}
        {selectedDeviceCam === 'WebRTC_env' && (
          <CameraWebRTCV3
            deviceId={selectedDeviceId}
            deviceIp={selectedDeviceIp}
            camera_src={selectedDeviceCamsrc}
            onClose={() => dispatch(devicesActions.selectId(null))}
          />
        )}
        {selectedDeviceCam === 'Websocket' && (
          <Camera
            deviceId={selectedDeviceId}
            datacamera={selectedImage}
            onClose={() => dispatch(devicesActions.selectId(null))}
          />
        )}

        <Toast toastlist={list} position="buttom-right" setList={setList} />
      </RosControl>
    </div>
  );
};

export default MainPage;
