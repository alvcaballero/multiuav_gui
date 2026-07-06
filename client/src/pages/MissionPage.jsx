import { useState, useEffect } from 'react';
import { useSelector } from 'react-redux';
import { Paper, Tabs, Tab, Box } from '@mui/material';
import { makeStyles } from 'tss-react/mui';

import MapView from '../map/core/MapView';
import Navbar from '../components/layout/Navbar';
import { Menu } from '../components/layout/Menu';
import { MapMissionsCreate } from '../map/draw/MapMissionsCreate';
import MapPositions from '../map/devices/MapPositions';
import MapMarkers from '../map/environment/MapMarkers';

import { RosControl } from '../components/commands/RosControl';
import MissionPanel from '../components/mission/MissionPanel';
import MissionElevation from '../components/mission/MissionElevation';
import MissionStats from '../components/mission/MissionStats';
import SaveFile from '../components/ui/SaveFile';
import MapScale from '../map/controls/MapScale';
import MapDefaultCamera from '../map/controls/MapDefaultCamera';
import MapMissionHome from '../map/mission/MapMissionHome';

const useStyles = makeStyles()((theme) => ({
  root: {
    margin: '0',
    height: '100vh',
  },
  sidebarStyle: {
    display: 'flex',
    flexDirection: 'column',
    position: 'fixed',
    left: 0,
    top: theme.dimensions.navbarHeight,
    height: 'calc(100% - 95px)',
    width: '560px',
    margin: '0px',
    zIndex: 3,
  },
  middleStyle: {
    flex: 1,
    display: 'grid',
  },
  panelElevation: {
    display: 'flex',
    flexDirection: 'column',
    position: 'fixed',
    right: 0,
    bottom: 0,
    height: '50vh',
    width: 'calc(100% - 560px)',
    margin: '0px',
    zIndex: 3,
  },
}));

const showToast = (type, description) => {
  // Toast notification placeholder
  console.log('Toast:', type, description);
};

const MissionPage = () => {
  const { classes } = useStyles();
  const [Opensave, setOpensave] = useState(false);
  const [bottomTab, setBottomTab] = useState(0);

  const positions = useSelector((state) => state.session.positions);
  const sessionmarkers = useSelector((state) => state.session.markers);

  const [filteredPositions, setFilteredPositions] = useState([]);
  const [markers, setMarkers] = useState([]);

  useEffect(() => {
    setFilteredPositions(Object.values(positions));
  }, [positions]);

  useEffect(() => {
    setMarkers(sessionmarkers);
  }, [sessionmarkers]);

  return (
    <div className={classes.root}>
      <RosControl notification={showToast}>
        <Navbar />
        <Menu />
        <div
          style={{
            float: 'right',
            width: 'calc(100% - 560px)',
            height: 'calc(70vh - 95px)',
            right: '0px',
            margin: 'auto',
          }}
        >
          <MapView>
            <MapMarkers markers={markers} />
            <MapDefaultCamera />
            <MapMissionHome />
            <MapMissionsCreate />
            <MapPositions
              positions={filteredPositions}
              onClick={null}
              selectedPosition={null}
              showStatus
            />
          </MapView>
          <MapScale />
        </div>

        <div className={classes.sidebarStyle}>
          <div className={classes.middleStyle}>
            <Paper square>
              <MissionPanel SetOpenSave={setOpensave} />
            </Paper>
          </div>
        </div>
        <div className={classes.panelElevation}>
          <Paper square sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
            <Tabs
              value={bottomTab}
              onChange={(_, newValue) => setBottomTab(newValue)}
              variant="fullWidth"
              sx={{ borderBottom: 1, borderColor: 'divider', minHeight: 36 }}
            >
              <Tab label="Statistics" sx={{ minHeight: 36, py: 0 }} />
              <Tab label="Elevation" sx={{ minHeight: 36, py: 0 }} />
            </Tabs>
            <Box sx={{ flex: 1, overflow: 'hidden', display: 'flex', flexDirection: 'column' }}>
              <Box
                sx={{
                  flex: 1,
                  overflow: 'hidden',
                  display: bottomTab === 0 ? 'flex' : 'none',
                  flexDirection: 'column',
                }}
              >
                <MissionStats />
              </Box>
              <Box
                sx={{
                  flex: 1,
                  overflow: 'hidden',
                  display: bottomTab === 1 ? 'flex' : 'none',
                  flexDirection: 'column',
                }}
              >
                <MissionElevation />
              </Box>
            </Box>
          </Paper>
        </div>
        {Opensave && <SaveFile SetOpenSave={setOpensave} />}
      </RosControl>
    </div>
  );
};

export default MissionPage;
