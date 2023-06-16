import './MainPage.css';
import React, { useState , useCallback, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Paper } from '@mui/material';
import { useTheme } from '@mui/material/styles';
//import { makeStyles } from '@mui/system';
import { Navbar } from './components/Navbar';
import { Menu } from './components/Menu';
import Toast from './components/Toast';
import { Adduav } from './components/Adduav';
import { Camera } from './components/Camera';
import { RosControl } from './components/RosControl';
import MapView from './Mapview/Mapview';
import MapPositions from './Mapview/MapPositions';
import MapMissions from './Mapview/MapMissions';
import MapSelectedDevice from './Mapview/MapSelectedDevice';
import DeviceList from './components/DeviceList';
import StatusCard from './components/StatusCard';
import MainToolbar from './components/MainToolbar';
import { CameraWebRTC } from './components/CameraWebRTC';

import { devicesActions } from './store';

const sidebarStyle= {
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
  };
  const middleStyle= {
    flex: 1,
    display: 'grid',
  };
  const contentListStyle= {
    pointerEvents: 'auto',
    gridArea: '1 / 1',
    zIndex: 4,
  };


const MainPage = () => {
  const dispatch = useDispatch();
  const theme = useTheme();


  const devices = useSelector((state) => state.devices.items);
  const positions = useSelector((state) => state.data.positions);
  const cameradata = useSelector((state) => state.data.camera);
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const [filteredPositions, setFilteredPositions] = useState([]);
  const [filteredImages, setFilteredImages] = useState([]);
  const selectedPosition = filteredPositions.find((position) => selectedDeviceId && position.deviceId === selectedDeviceId);
  const selectedImage = filteredImages.find((camera) => selectedDeviceId && camera.deviceId == selectedDeviceId)
  
  let listdevices =Object.values(devices);

  const [AddUAVOpen, SetAddUAVOpen] = useState(false);

  const [list, setList] = useState([]);
  let toastProperties = null;
  const showToast = (type,description)=> {
    switch(type) {
      case 'success':
        toastProperties = {
          id: list.length+1,
          title: 'Success',
          description: description,
          backgroundColor: '#5cb85c'
        }
        break;
      case 'danger':
        toastProperties = {
          id: list.length+1,
          title: 'Danger',
          description: description,
          backgroundColor: '#d9534f'
        }
        break;
      case 'info':
        toastProperties = {
          id: list.length+1,
          title: 'Info',
          description: description,
          backgroundColor: '#5bc0de'
        }
        break;
      case 'warning':
        toastProperties = {
          id: list.length+1,
          title: 'Warning',
          description: description,
          backgroundColor: '#f0ad4e'
        }
        break;
      default:
        toastProperties = [];
      }
      setList([...list, toastProperties]);
  }




  const onMarkerClick = useCallback((_, deviceId) => {
    dispatch(devicesActions.selectId(deviceId));
  }, [dispatch]);


  useEffect(() => {
  setFilteredPositions(Object.values(positions))
  }, [positions]); 
  useEffect(() => {
    setFilteredImages(Object.values(cameradata))
  }, [cameradata]); 

  return (
    <div className="App">
      <div className="window">
        <RosControl notification={showToast}>
          <Navbar SetAddUAVOpen={SetAddUAVOpen} />
          <Menu SetAddUAVOpen={SetAddUAVOpen} />
          <MapView>
            {AddUAVOpen && <Adduav SetAddUAVOpen={SetAddUAVOpen} />}
            <MapMissions/>
            <MapPositions positions={filteredPositions} onClick={onMarkerClick} selectedPosition={selectedPosition} showStatus />
            <MapSelectedDevice/>
          </MapView>
          <div style={sidebarStyle}>
              <div style={middleStyle}>
                <Paper square style={contentListStyle} >
                  <MainToolbar/>
                  <DeviceList devices={listdevices} />
                </Paper>
              </div>
          </div>
          <StatusCard
          deviceId={selectedDeviceId}
          position={selectedPosition}
          onClose={() => dispatch(devicesActions.selectId(null))}
          desktopPadding={theme.dimensions.drawerWidthDesktop}
          />
          {false && <CameraWebRTC
          deviceId={selectedDeviceId}
          position={selectedPosition}
          onClose={() => dispatch(devicesActions.selectId(null))}
          />}
          {<Camera
          deviceId={selectedDeviceId}
          datacamera={selectedImage}
          onClose={() => dispatch(devicesActions.selectId(null))}
          />}
          <Toast toastlist={list} position="buttom-right" setList={setList} />
        </RosControl>
      </div>

    </div>
  );
};

export default MainPage;
