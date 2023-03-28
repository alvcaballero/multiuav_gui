import './App.css';
import React, { useState , useCallback, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Paper } from '@mui/material';
import { useTheme } from '@mui/material/styles';
import { makeStyles } from '@mui/system';
import { Navbar } from './components/Navbar';
import { Menu } from './components/Menu';
import './assets/css/photon.min.css'
import { Overlaytab } from './components/Overlaytab';
import { Statuswindow } from './components/Statuswindow';
import { Adduav } from './components/Adduav';
import { Camera } from './components/Camera';
import { RosControl } from './components/RosControl';
import MapView from './Mapview/Mapview';
import MapPositions from './Mapview/MapPositions';
import DeviceList from './components/DeviceList';
import StatusCard from './components/StatusCard';

import { devicesActions } from './store';


const sidebarStyle= {
    pointerEvents: 'none',
    display: 'flex',
    flexDirection: 'column',
    position: 'fixed',
    left: 0,
    top: '100px',
    height: `calc(100% - 200px)`,
    width: '360px',
    margin: '20px',
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


function App() {
  const dispatch = useDispatch();
  const theme = useTheme();


  const devices = useSelector((state) => state.devices.items);
  const positions = useSelector((state) => state.data.positions);
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const [filteredPositions, setFilteredPositions] = useState([]);
  const selectedPosition = filteredPositions.find((position) => selectedDeviceId && position.deviceId === selectedDeviceId);
  
  let listdevices =Object.values(devices);

  const [AddUAVOpen, SetAddUAVOpen] = useState(false);

  const onMarkerClick = useCallback((_, deviceId) => {
    dispatch(devicesActions.selectId(deviceId));
  }, [dispatch]);


  useEffect(() => {
  setFilteredPositions(Object.values(positions))
  }, [positions]); 

  return (
    <div className="App">
      <div className="window">
        <RosControl>
          <Navbar SetAddUAVOpen={SetAddUAVOpen} />
          <Menu SetAddUAVOpen={SetAddUAVOpen} />
          <MapView>
            {AddUAVOpen && <Adduav SetAddUAVOpen={SetAddUAVOpen} />}
            <MapPositions positions={filteredPositions} onClick={onMarkerClick} selectedPosition={selectedPosition} showStatus />
            <Statuswindow/>
            {/*<Overlaytab/>
            <Camera/>*/}
          </MapView>
          <div style={sidebarStyle}>
              <div style={middleStyle}>
                <Paper square style={contentListStyle} >
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
        </RosControl>
      </div>

    </div>
  );
}

export default App;
