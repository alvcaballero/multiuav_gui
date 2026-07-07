import { useState } from 'react';
import { useNavigate, useLocation } from 'react-router-dom';
import { useSelector, useDispatch } from 'react-redux';
import { makeStyles } from 'tss-react/mui';
import { grey, red, orange, green } from '@mui/material/colors';

import { map } from '../../map/core/mapInstance';
import { usePreference } from '../../shared/preferences';
import { useMissionFile } from '../../services/useMissionFile';
import {
  commandLoadMission,
  commandMission,
  commandStopMission,
  commandPauseMission,
  commandResumeMission,
} from '../../shared/fetchs';
import { useCatch } from '../../reactHelper';
import {
  missionActions,
  activeMissionsActions,
  sessionActions,
  getCommandableMissionId,
} from '../../store';
import { getMissionCentroid } from '../../shared/util/missionGeo';
import SwipeConfirm from '../../shared/components/SwipeConfirm';
import MissionDetailPopover from './menu/MissionDetailPopover';
import ActiveMissionsPopover from './menu/ActiveMissionsPopover';
import EventsPopover from './menu/EventsPopover';
import MenuLeftGroup from './menu/MenuLeftGroup';
import MenuCenterStatus from './menu/MenuCenterStatus';
import MenuRightGroup from './menu/MenuRightGroup';

const useStyles = makeStyles()(() => ({
  toolbar: {
    height: '36px',
    background: 'linear-gradient(180deg, #f0f2f5 0%, #e4e7ec 100%)',
    borderBottom: '1px solid #c8cdd6',
    boxShadow: '0 1px 3px rgba(0,0,0,0.08)',
    display: 'flex',
    alignItems: 'center',
    flexShrink: 0,
    userSelect: 'none',
  },
  leftGroup: {
    display: 'flex',
    alignItems: 'center',
    gap: '1px',
    paddingLeft: '6px',
    flex: '1 1 0',
    minWidth: 0,
  },
  rightGroup: {
    display: 'flex',
    alignItems: 'center',
    gap: '2px',
    paddingRight: '8px',
    flex: '1 1 0',
    minWidth: 0,
    justifyContent: 'flex-end',
  },
  center: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    flexShrink: 0,
    padding: '0 8px',
  },
  vDivider: {
    width: '1px',
    height: '18px',
    backgroundColor: '#c0c5ce',
    margin: '0 5px',
    flexShrink: 0,
  },
  iconBtn: {
    color: grey[600],
    padding: '4px',
    borderRadius: '5px',
    '&:hover': { backgroundColor: 'rgba(0,0,0,0.07)', color: grey[900] },
    '&.Mui-disabled': { color: grey[400] },
  },
  stopBtn: {
    color: red[700],
    padding: '4px',
    borderRadius: '5px',
    '&:hover': { backgroundColor: red[50] },
    '&.Mui-disabled': { color: grey[400] },
  },
  pauseBtn: {
    color: orange[800],
    padding: '4px',
    borderRadius: '5px',
    '&:hover': { backgroundColor: orange[50] },
    '&.Mui-disabled': { color: grey[400] },
  },
  resumeBtn: {
    color: green[700],
    padding: '4px',
    borderRadius: '5px',
    '&:hover': { backgroundColor: green[50] },
    '&.Mui-disabled': { color: grey[400] },
  },
  rosChip: {
    height: '20px',
    fontSize: '10px',
    fontWeight: 600,
    letterSpacing: '0.3px',
    cursor: 'default',
    flexShrink: 0,
  },
  loadBtn: {
    height: '24px',
    fontSize: '11px',
    fontWeight: 600,
    textTransform: 'none',
    borderRadius: '5px',
    paddingLeft: '8px',
    paddingRight: '8px',
    color: grey[700],
    border: `1px solid ${grey[400]}`,
    '&:hover': { backgroundColor: grey[200], border: `1px solid ${grey[500]}` },
    '&.Mui-disabled': { color: grey[400], border: `1px solid ${grey[300]}` },
  },
  flyBtn: {
    height: '24px',
    fontSize: '11px',
    fontWeight: 600,
    textTransform: 'none',
    borderRadius: '5px',
    paddingLeft: '8px',
    paddingRight: '8px',
    background: 'linear-gradient(135deg, #1565c0 0%, #0d47a1 100%)',
    color: '#fff',
    border: 'none',
    '&:hover': { background: 'linear-gradient(135deg, #1976d2 0%, #1565c0 100%)' },
    '&.Mui-disabled': { background: grey[300], color: grey[500] },
  },
  toggleBtn: {
    height: '24px',
    fontSize: '11px',
    fontWeight: 600,
    textTransform: 'none',
    borderRadius: '5px',
    paddingLeft: '7px',
    paddingRight: '7px',
    color: grey[700],
    border: `1px solid ${grey[400]}`,
    '&:hover': { backgroundColor: grey[200], border: `1px solid ${grey[500]}` },
  },
}));

export const Menu = () => {
  const { classes } = useStyles();
  const navigate = useNavigate();
  const dispatch = useDispatch();
  const location = useLocation();
  const is3D = location.pathname === '/3Dview';

  const handleMissionFile = useMissionFile();

  const mission = useSelector((state) => state.mission);
  const missionName = useSelector((state) => state.mission.name);
  const missionHome = useSelector((state) => state.mission.home);
  const socketState = useSelector((state) => state.session.socket);
  const devices = useSelector((state) => state.devices.items);
  const activeMissions = useSelector((state) => state.activeMissions.items);
  const selectedMissionId = useSelector((state) => state.activeMissions.selectedMissionId);
  const events = useSelector((state) => state.events.items);

  const defaultLatitude = usePreference('latitude');
  const defaultLongitude = usePreference('longitude');
  const defaultZoom = usePreference('zoom', 10);

  const commandableMissionId = useSelector(getCommandableMissionId);
  // Guards against double-click/double-tap firing loadMission twice in a row,
  // which would create duplicate Mission/Plan/Route rows server-side and
  // re-send configureMission to the same drones for no reason.
  const [loadingMission, setLoadingMission] = useState(false);
  const handleLoadMission = useCatch(async () => {
    if (loadingMission) return;
    setLoadingMission(true);
    try {
      const res = await commandLoadMission(mission);
      // Select the created mission so it becomes the one commandMission will command.
      if (res?.missionId != null) dispatch(activeMissionsActions.selectMission(res.missionId));
      return res;
    } finally {
      setLoadingMission(false);
    }
  });
  const handleCommandMission = useCatch(() => commandMission(commandableMissionId));
  const handleStopMission = useCatch(() => commandStopMission(devices));
  const handlePauseMission = useCatch(() => commandPauseMission(devices));
  const handleResumeMission = useCatch(() => commandResumeMission(devices));

  const LOADED_NAMES = ['Mission no loaded', 'no load mission'];
  const hasMission = Boolean(missionName && !LOADED_NAMES.includes(missionName));

  // Status shown reflects the selected mission; fall back to any running one so
  // the controls stay meaningful when nothing is explicitly selected.
  const selectedMission = selectedMissionId != null ? activeMissions[selectedMissionId] : null;
  const currentStatus = selectedMission?.status ?? null;
  const isRunning = currentStatus === 'running';

  const [eventsLastSeen, setEventsLastSeen] = useState(() => Date.now());
  const unseenErrors = events.filter((e) => e.type === 'error' && e.eventTime > eventsLastSeen);

  const readFile = (e) => handleMissionFile(e.target.files[0]);

  function goHome() {
    if (is3D || !map) return;
    map.easeTo({
      center: [defaultLongitude, defaultLatitude],
      zoom: Math.max(map.getZoom(), defaultZoom),
      offset: [0, -1 / 2],
    });
  }

  function goToMission() {
    if (is3D || !map || !missionHome) return;
    map.easeTo({
      center: [missionHome[1], missionHome[0]],
      zoom: Math.max(map.getZoom(), defaultZoom),
      offset: [0, -1 / 2],
    });
  }

  // Centra el origen 3D en el centroide de la misión actual antes de abrir la vista 3D,
  // igual que hace el Pegman al soltarse sobre un punto del mapa 2D.
  function goto3DView() {
    const centroid = getMissionCentroid(mission.route);
    if (centroid) {
      dispatch(sessionActions.updateScene3dOrigin(centroid));
    }
    navigate('/3Dview');
  }

  const [missionsAnchor, setMissionsAnchor] = useState(null);
  const [eventsAnchor, setEventsAnchor] = useState(null);
  const [missionDetailAnchor, setMissionDetailAnchor] = useState(null);
  const [confirmFly, setConfirmFly] = useState(false);
  const [confirmClear, setConfirmClear] = useState(false);

  function openEvents(e) {
    setEventsAnchor(e.currentTarget);
    setEventsLastSeen(Date.now());
  }

  return (
    <header className={classes.toolbar}>
      <MenuLeftGroup
        classes={classes}
        socketState={socketState}
        is3D={is3D}
        hasMission={hasMission}
        onGoHome={goHome}
        onOpenFile={readFile}
        onClearMission={() => setConfirmClear(true)}
      />

      <MenuCenterStatus
        classes={classes}
        hasMission={hasMission}
        is3D={is3D}
        missionName={missionName}
        currentStatus={currentStatus}
        onGoToMission={goToMission}
        onOpenMissionDetail={(e) => setMissionDetailAnchor(e.currentTarget)}
      />

      <MenuRightGroup
        classes={classes}
        is3D={is3D}
        hasMission={hasMission}
        loadingMission={loadingMission}
        isRunning={isRunning}
        unseenErrorsCount={unseenErrors.length}
        onLoadMission={handleLoadMission}
        onRequestFly={() => setConfirmFly(true)}
        onPauseMission={handlePauseMission}
        onResumeMission={handleResumeMission}
        onStopMission={handleStopMission}
        onNavigatePlanning={() => navigate('/planning')}
        onNavigateMission={() => navigate('/mission')}
        onToggleView={() => (is3D ? navigate('/') : goto3DView())}
        onOpenActiveMissions={(e) => setMissionsAnchor(e.currentTarget)}
        onOpenEvents={openEvents}
      />

      {/* ── Popovers ── */}
      <MissionDetailPopover
        anchor={missionDetailAnchor}
        onClose={() => setMissionDetailAnchor(null)}
        onClear={() => setConfirmClear(true)}
      />
      <ActiveMissionsPopover anchor={missionsAnchor} onClose={() => setMissionsAnchor(null)} />
      <EventsPopover anchor={eventsAnchor} onClose={() => setEventsAnchor(null)} />

      {/* ── Confirmaciones ── */}
      <SwipeConfirm
        enable={confirmFly}
        onClose={() => setConfirmFly(false)}
        onSucces={() => {
          setConfirmFly(false);
          handleCommandMission();
        }}
      />
      <SwipeConfirm
        enable={confirmClear}
        onClose={() => setConfirmClear(false)}
        onSucces={() => {
          setConfirmClear(false);
          dispatch(missionActions.clearMission());
        }}
      />
    </header>
  );
};
