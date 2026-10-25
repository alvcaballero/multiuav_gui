import React, { useState, useCallback, useMemo } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Paper } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import { useTheme } from '@mui/material/styles';
import Navbar from '../components/layout/Navbar';
import { Menu } from '../components/layout/Menu';
import AddDevice from '../components/devices/AddDevice';
import { commandMission } from '../common/util/fetchs';
import { useCatch } from '../reactHelper';

import DeviceList from '../components/devices/DeviceList';
import SwipeConfirm from '../common/components/SwipeConfirm';
import MainToolbar from '../components/layout/MainToolbar';
import StatusCard from '../components/devices/StatusCard';
import CameraDevicePanel from '../components/camera/CameraDevicePanel';

import { devicesActions, getCommandableMissionId } from '../store';
import useFilter from '../components/devices/useFilter';
import usePersistedState from '../common/util/usePersistedState';

import Scene3DCanvas from '../scene3d/Scene3DCanvas';

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
    top: theme.dimensions.navbarHeight,
    height: `calc(100% - ${theme.dimensions.navbarHeight})`,
    width: theme.dimensions.drawerWidthDesktop,
    margin: '0px',
    zIndex: 3,
  },
  canvas: {
    position: 'absolute',
    top: theme.dimensions.navbarHeight,
    right: '0px',
    width: `calc(100% - ${theme.dimensions.drawerWidthDesktop})`,
    height: `calc(100% - ${theme.dimensions.navbarHeight})`,
  },
}));

const MainPage3D = () => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const theme = useTheme();

  const commandableMissionId = useSelector(getCommandableMissionId);
  const positions = useSelector((state) => state.session.positions);
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);

  const [keyword, setKeyword] = useState('');
  const [filter, setFilter] = usePersistedState('filter', {
    statuses: [],
    vehicleTypes: [],
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

  const selectedPosition = useMemo(
    () => filteredPositions.find((p) => selectedDeviceId && p.deviceId === selectedDeviceId),
    [filteredPositions, selectedDeviceId],
  );

  const handleCommandMission = useCatch(() => commandMission(commandableMissionId));

  const [addDeviceOpen, setAddDeviceOpen] = useState(false);
  const [confirmMission, setConfirmMission] = useState(false);

  const unselectDevice = useCallback(() => {
    dispatch(devicesActions.selectId(null));
  }, [dispatch]);

  return (
    <div className={classes.root}>
      <Navbar setAddDeviceOpen={setAddDeviceOpen} setconfirmMission={setConfirmMission} />
      <Menu />
      <Scene3DCanvas className={classes.canvas} />

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
            setAddDeviceOpen={setAddDeviceOpen}
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
          is3d={true}
        />
      )}
      <SwipeConfirm
        enable={confirmMission}
        onClose={() => setConfirmMission(false)}
        onSucces={() => handleCommandMission()}
      />
      <CameraDevicePanel deviceId={selectedDeviceId} onClose={unselectDevice} />
      {addDeviceOpen && <AddDevice onClose={() => setAddDeviceOpen(false)} />}
    </div>
  );
};

export default MainPage3D;
