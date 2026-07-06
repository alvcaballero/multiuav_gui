import { useState } from 'react';
import { useNavigate, useLocation } from 'react-router-dom';
import { useSelector, useDispatch } from 'react-redux';
import { makeStyles } from 'tss-react/mui';
import { grey, red, orange, green } from '@mui/material/colors';
import { missionStyle } from '../../shared/missionStatus';

import HomeIcon from '@mui/icons-material/Home';
import FolderOpenIcon from '@mui/icons-material/FolderOpen';
import FileUploadIcon from '@mui/icons-material/FileUpload';
import SendIcon from '@mui/icons-material/Send';
import ModeEditIcon from '@mui/icons-material/ModeEdit';
import MapIcon from '@mui/icons-material/Map';
import ViewInArIcon from '@mui/icons-material/ViewInAr';
import AssignmentIcon from '@mui/icons-material/Assignment';
import NotificationsIcon from '@mui/icons-material/Notifications';
import CircleIcon from '@mui/icons-material/Circle';
import DeleteForeverIcon from '@mui/icons-material/DeleteForever';
import StopIcon from '@mui/icons-material/Stop';
import PauseIcon from '@mui/icons-material/Pause';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import RouteIcon from '@mui/icons-material/Route';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import MyLocationIcon from '@mui/icons-material/MyLocation';

import { Button, IconButton, Box, Chip, Tooltip, Typography } from '@mui/material';

import { map } from '../../map/core/MapView';
import { RosContext } from '../commands/RosControl';
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
      {/* ── LEFT: conexión · mapa · archivo ── */}
      <div className={classes.leftGroup}>
        <Tooltip
          title={socketState ? 'WebSocket connected' : 'WebSocket disconnected'}
          placement="bottom"
        >
          <Chip
            icon={
              <CircleIcon
                sx={{ fontSize: '7px !important', color: socketState ? green[500] : red[500] }}
              />
            }
            label={socketState ? 'WS' : 'offline'}
            size="small"
            className={classes.rosChip}
            sx={{
              backgroundColor: socketState ? 'rgba(76,175,80,0.12)' : 'rgba(244,67,54,0.10)',
              color: socketState ? green[800] : red[700],
              border: `1px solid ${socketState ? 'rgba(76,175,80,0.35)' : 'rgba(244,67,54,0.35)'}`,
            }}
          />
        </Tooltip>

        <div className={classes.vDivider} />

        <RosContext.Consumer>
          {({ rosState }) => (
            <Chip
              icon={
                <CircleIcon
                  sx={{ fontSize: '7px !important', color: rosState ? green[500] : red[500] }}
                />
              }
              label={rosState ? 'ROS' : 'offline'}
              size="small"
              className={classes.rosChip}
              sx={{
                backgroundColor: rosState ? 'rgba(76,175,80,0.12)' : 'rgba(244,67,54,0.10)',
                color: rosState ? green[800] : red[700],
                border: `1px solid ${rosState ? 'rgba(76,175,80,0.35)' : 'rgba(244,67,54,0.35)'}`,
              }}
            />
          )}
        </RosContext.Consumer>

        <div className={classes.vDivider} />

        <Tooltip title="Home position" placement="bottom">
          <span>
            <IconButton className={classes.iconBtn} size="small" onClick={goHome} disabled={is3D}>
              <HomeIcon sx={{ fontSize: 17 }} />
            </IconButton>
          </span>
        </Tooltip>

        <Tooltip title="Open mission file" placement="bottom">
          <IconButton className={classes.iconBtn} size="small" component="label">
            <FolderOpenIcon sx={{ fontSize: 17 }} />
            <input type="file" multiple={false} style={{ display: 'none' }} onChange={readFile} />
          </IconButton>
        </Tooltip>

        <Tooltip title="Clear mission" placement="bottom">
          <span>
            <IconButton
              className={classes.iconBtn}
              size="small"
              onClick={() => setConfirmClear(true)}
              disabled={!hasMission}
              sx={{ '&:hover': { color: red[600], backgroundColor: red[50] } }}
            >
              <DeleteForeverIcon sx={{ fontSize: 17 }} />
            </IconButton>
          </span>
        </Tooltip>
      </div>

      {/* ── CENTER: localizar · nombre · status (grupo unificado) ── */}
      <div className={classes.center}>
        <Box
          sx={{
            display: 'flex',
            alignItems: 'stretch',
            height: '26px',
            border: `1px solid ${grey[300]}`,
            borderRadius: '6px',
            overflow: 'hidden',
            backgroundColor: '#fff',
            boxShadow: '0 1px 2px rgba(0,0,0,0.06)',
          }}
        >
          {/* Segmento localizar */}
          <Tooltip title={hasMission && !is3D ? 'Center map on mission' : ''} placement="bottom">
            <span style={{ display: 'flex' }}>
              <Box
                component="button"
                onClick={hasMission && !is3D ? goToMission : undefined}
                disabled={!hasMission || is3D}
                sx={{
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  width: '28px',
                  border: 'none',
                  borderRight: `1px solid ${grey[200]}`,
                  backgroundColor: 'transparent',
                  padding: 0,
                  cursor: hasMission && !is3D ? 'pointer' : 'default',
                  color: hasMission && !is3D ? grey[600] : grey[400],
                  '&:hover':
                    hasMission && !is3D ? { backgroundColor: grey[100], color: grey[900] } : {},
                }}
              >
                <MyLocationIcon sx={{ fontSize: 14 }} />
              </Box>
            </span>
          </Tooltip>

          {/* Segmento nombre */}
          <Tooltip title={hasMission ? 'Mission details' : ''} placement="bottom">
            <span style={{ display: 'flex' }}>
              <Box
                component="button"
                onClick={hasMission ? (e) => setMissionDetailAnchor(e.currentTarget) : undefined}
                disabled={!hasMission}
                sx={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: '5px',
                  border: 'none',
                  borderRight: `1px solid ${grey[200]}`,
                  backgroundColor: 'transparent',
                  padding: '0 10px',
                  minWidth: '160px',
                  maxWidth: '300px',
                  cursor: hasMission ? 'pointer' : 'default',
                  '&:hover': hasMission ? { backgroundColor: grey[50] } : {},
                }}
              >
                <Typography
                  sx={{
                    fontSize: '12px',
                    fontWeight: 600,
                    overflow: 'hidden',
                    textOverflow: 'ellipsis',
                    whiteSpace: 'nowrap',
                    color: hasMission ? grey[800] : grey[400],
                    flex: 1,
                    textAlign: 'left',
                  }}
                >
                  {hasMission ? missionName : 'No mission loaded'}
                </Typography>
                {hasMission && (
                  <ExpandMoreIcon sx={{ fontSize: 14, color: grey[400], flexShrink: 0 }} />
                )}
              </Box>
            </span>
          </Tooltip>

          {/* Segmento status */}
          <Box
            sx={{
              display: 'flex',
              alignItems: 'center',
              gap: '5px',
              padding: '0 9px',
              minWidth: '72px',
              justifyContent: 'center',
              backgroundColor: currentStatus ? missionStyle(currentStatus).color : grey[100],
              cursor: 'default',
            }}
          >
            {currentStatus && (
              <CircleIcon
                sx={{
                  fontSize: 7,
                  color: '#fff',
                  flexShrink: 0,
                  ...(currentStatus === 'running' && {
                    animation: 'menuPulse 1.4s ease-in-out infinite',
                    '@keyframes menuPulse': { '0%,100%': { opacity: 1 }, '50%': { opacity: 0.3 } },
                  }),
                }}
              />
            )}
            <Typography
              sx={{
                fontSize: '10px',
                fontWeight: 700,
                letterSpacing: '0.4px',
                textTransform: 'uppercase',
                whiteSpace: 'nowrap',
                color: currentStatus ? '#fff' : grey[500],
              }}
            >
              {currentStatus ? missionStyle(currentStatus).label : 'no status'}
            </Typography>
          </Box>
        </Box>
      </div>

      {/* ── RIGHT: comandos · control · vista · monitoreo ── */}
      <div className={classes.rightGroup}>
        <Tooltip title="Load mission to UAVs" placement="bottom">
          <span>
            <Button
              className={classes.loadBtn}
              startIcon={<FileUploadIcon sx={{ fontSize: 14 }} />}
              onClick={handleLoadMission}
              disabled={!hasMission || loadingMission}
              variant="outlined"
            >
              {loadingMission ? 'Loading…' : 'Load'}
            </Button>
          </span>
        </Tooltip>

        <Tooltip title="Command mission — Fly!" placement="bottom">
          <span>
            <Button
              className={classes.flyBtn}
              startIcon={<SendIcon sx={{ fontSize: 13 }} />}
              onClick={() => setConfirmFly(true)}
              disabled={!hasMission}
            >
              Fly!
            </Button>
          </span>
        </Tooltip>

        <div className={classes.vDivider} />

        <Tooltip title="Pause mission" placement="bottom">
          <span>
            <IconButton
              className={classes.pauseBtn}
              size="small"
              onClick={handlePauseMission}
              disabled={!isRunning}
            >
              <PauseIcon sx={{ fontSize: 17 }} />
            </IconButton>
          </span>
        </Tooltip>

        <Tooltip title="Resume mission" placement="bottom">
          <span>
            <IconButton
              className={classes.resumeBtn}
              size="small"
              onClick={handleResumeMission}
              disabled={!isRunning}
            >
              <PlayArrowIcon sx={{ fontSize: 17 }} />
            </IconButton>
          </span>
        </Tooltip>

        <Tooltip title="Stop mission" placement="bottom">
          <span>
            <IconButton
              className={classes.stopBtn}
              size="small"
              onClick={handleStopMission}
              disabled={!isRunning}
            >
              <StopIcon sx={{ fontSize: 17 }} />
            </IconButton>
          </span>
        </Tooltip>

        <div className={classes.vDivider} />

        <Tooltip title="Planning" placement="bottom">
          <IconButton
            className={classes.iconBtn}
            size="small"
            onClick={() => navigate('/planning')}
          >
            <RouteIcon sx={{ fontSize: 17 }} />
          </IconButton>
        </Tooltip>

        <Tooltip title="Edit mission" placement="bottom">
          <IconButton className={classes.iconBtn} size="small" onClick={() => navigate('/mission')}>
            <ModeEditIcon sx={{ fontSize: 17 }} />
          </IconButton>
        </Tooltip>

        <Tooltip title={is3D ? 'Switch to 2D map' : 'Switch to 3D view'} placement="bottom">
          <Button
            className={classes.toggleBtn}
            startIcon={
              is3D ? <MapIcon sx={{ fontSize: 13 }} /> : <ViewInArIcon sx={{ fontSize: 13 }} />
            }
            onClick={() => (is3D ? navigate('/') : goto3DView())}
          >
            {is3D ? '2D' : '3D'}
          </Button>
        </Tooltip>

        <div className={classes.vDivider} />

        <Tooltip title="Active missions" placement="bottom">
          <IconButton
            className={classes.iconBtn}
            size="small"
            onClick={(e) => setMissionsAnchor(e.currentTarget)}
          >
            <AssignmentIcon sx={{ fontSize: 17 }} />
          </IconButton>
        </Tooltip>

        <Tooltip
          title={unseenErrors.length ? `${unseenErrors.length} new error(s)` : 'Events log'}
          placement="bottom"
        >
          <IconButton
            className={classes.iconBtn}
            size="small"
            onClick={openEvents}
            sx={{ position: 'relative' }}
          >
            <NotificationsIcon sx={{ fontSize: 17 }} />
            {unseenErrors.length > 0 && (
              <Box
                sx={{
                  position: 'absolute',
                  top: 3,
                  right: 3,
                  width: 7,
                  height: 7,
                  borderRadius: '50%',
                  backgroundColor: red[500],
                  border: '1.5px solid #e4e7ec',
                }}
              />
            )}
          </IconButton>
        </Tooltip>
      </div>

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
