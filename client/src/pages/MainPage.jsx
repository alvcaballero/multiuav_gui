import { useState, useCallback, useMemo, Suspense, lazy } from 'react';
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
import MainMap from '../map/MainMap';
import StatusCard from '../components/devices/StatusCard';
import CameraDevice from '../components/camera/CameraDevice';
import { devicesActions, getCommandableMissionId } from '../store';
import useFilter from '../components/devices/useFilter';
import usePersistedState from '../shared/usePersistedState';

const ChatDrawer = lazy(() => import('../components/chat/ChatDrawer'));

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
    top: theme.dimensions.navbarHeight,
    bottom: 0,
    width: theme.dimensions.drawerWidthDesktop,
    margin: '0px',
    zIndex: 3,
  },
  map: {
    position: 'absolute',
    top: theme.dimensions.navbarHeight,
    left: theme.dimensions.drawerWidthDesktop,
    right: '0px',
    bottom: '0px',
  },
}));

const MainPage = () => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const theme = useTheme();

  const llmEnabled = useSelector((state) => state.session.server?.llmEnabled ?? false);
  const positions = useSelector((state) => state.session.positions);
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const sessionMarkers = useSelector((state) => state.session.markers);
  const routes = useSelector((state) => state.mission.route);

  const selectedPosition = useMemo(
    () => filteredPositions.find((p) => selectedDeviceId && p.deviceId === selectedDeviceId),
    [filteredPositions, selectedDeviceId],
  );

  const [keyword, setKeyword] = useState('');
  const [filter, setFilter] = usePersistedState('filter', {
    statuses: [],
    groups: [],
  });
  const [filterSort, setFilterSort] = usePersistedState('filterSort', '');
  const [filterMap, setFilterMap] = usePersistedState('filterMap', false);

  const { filteredDevices, filteredPositions } = useFilter(
    keyword,
    filter,
    filterSort,
    filterMap,
    positions,
  );

  const commandableMissionId = useSelector(getCommandableMissionId);
  const handleCommandMission = useCatch(() => commandMission(commandableMissionId));

  const [AddUAVOpen, setAddUAVOpen] = useState(false);
  const [chatOpen, setChatOpen] = useState(false);
  const [confirmMission, setConfirmMission] = useState(false);

  const unselectDevice = useCallback(() => {
    dispatch(devicesActions.selectId(null));
  }, [dispatch]);

  return (
    <div className={classes.root}>
      <Navbar
        SetAddUAVOpen={setAddUAVOpen}
        setconfirmMission={setConfirmMission}
        setChatOpen={setChatOpen}
      />
      <RosControl>
        <Menu />
      </RosControl>

      <SwipeConfirm
        enable={confirmMission}
        onClose={() => setConfirmMission(false)}
        onSucces={() => handleCommandMission()}
      />
      <div className={classes.map}>
        <MainMap
          filteredPositions={filteredPositions}
          markers={sessionMarkers}
          routes={routes}
          selectedPosition={selectedPosition}
        />
      </div>
      <div className={classes.sidebar}>
        <Paper square elevation={3} className={classes.header}>
          <MainToolbar
            filteredDevices={filteredDevices}
            keyword={keyword}
            setKeyword={setKeyword}
            filter={filter}
            setFilter={setFilter}
            filterSort={filterSort}
            setFilterSort={setFilterSort}
            filterMap={filterMap}
            setFilterMap={setFilterMap}
            SetAddUAVOpen={setAddUAVOpen}
          />
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
      {llmEnabled && (
        <Suspense fallback={null}>
          <ChatDrawer open={chatOpen} onClose={() => setChatOpen(false)} />
        </Suspense>
      )}
      <CameraDevice deviceId={selectedDeviceId} onClose={unselectDevice} />
      {AddUAVOpen && <Adduav SetAddUAVOpen={setAddUAVOpen} />}
    </div>
  );
};

export default MainPage;
