//import "./MainPage.css";
import React, { useContext, useState, useCallback, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Paper } from '@mui/material';
import { useTheme } from '@mui/material/styles';
import makeStyles from '@mui/styles/makeStyles';
import { Navbar } from '../components/Navbar';
import { Menu } from '../components/Menu';
import Toast from '../components/Toast';
import { Adduav } from '../components/Adduav';
import { Camera } from '../components/Camera';
import { RosControl, RosContext } from '../components/RosControl';
import MapView from '../Mapview/MapView';
import MapPositions from '../Mapview/MapPositions';
import MapMissions from '../Mapview/MapMissions';
import MapMarkers from '../Mapview/MapMarkers';
import MapSelectedDevice from '../Mapview/MapSelectedDevice';
import DeviceList from '../components/DeviceList';
import StatusCard from '../components/StatusCard';
import SwipeConfirm from '../common/components/SwipeConfirm';
import MainToolbar from '../components/MainToolbar';
import { CameraWebRTCV3 } from '../components/CameraWebRTCV3';

import { devicesActions } from '../store';

const useStyles = makeStyles((theme) => ({
  root: {
    height: '100%',
  },
  sidebarStyle: {
    pointerEvents: 'none',
    display: 'flex',
    flexDirection: 'column',
    position: 'fixed',
    left: 0,
    top: '88px',
    height: `calc(100% - 88px)`,
    width: '360px',
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
    switch (type) {
      case 'success':
        toastProperties = {
          id: list.length + 1,
          title: 'Success',
          description: description,
          backgroundColor: '#5cb85c',
        };
        break;
      case 'danger':
        toastProperties = {
          id: list.length + 1,
          title: 'Danger',
          description: description,
          backgroundColor: '#d9534f',
        };
        break;
      case 'info':
        toastProperties = {
          id: list.length + 1,
          title: 'Info',
          description: description,
          backgroundColor: '#5bc0de',
        };
        break;
      case 'warning':
        toastProperties = {
          id: list.length + 1,
          title: 'Warning',
          description: description,
          backgroundColor: '#f0ad4e',
        };
        break;
      default:
        toastProperties = [];
    }
    setList([...list, toastProperties]);
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
            width: '100%',
            height: `calc(100vh - 88px)`,
          }}
        >
          <MapView>
            <MapMarkers markers={markers} />
            <MapMissions />

            <MapPositions
              positions={filteredPositions}
              onClick={onMarkerClick}
              selectedPosition={selectedPosition}
              showStatus
            />
            <MapSelectedDevice />
          </MapView>
        </div>

        <div className={classes.sidebarStyle}>
          <div className={classes.middleStyle}>
            <Paper square className={classes.contentListStyle}>
              <MainToolbar SetAddUAVOpen={SetAddUAVOpen} />
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

        <Toast toastlist={list} position='buttom-right' setList={setList} />
      </RosControl>
    </div>
  );
};

export default MainPage;
