import { lazy, Suspense, useEffect, useRef } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import {
  Button,
  IconButton,
  Paper,
  ToggleButton,
  ToggleButtonGroup,
  Typography,
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import CloseIcon from '@mui/icons-material/Close';
import CameraAltIcon from '@mui/icons-material/CameraAlt';
import RadarIcon from '@mui/icons-material/Radar';
import GpsFixedIcon from '@mui/icons-material/GpsFixed';
import dayjs from 'dayjs';

import MainMap from '../../map/MainMap';
import MapGhostTarget from '../../map/devices/MapGhostTarget';
import CameraDevice from '../../components/camera/CameraDevice';
import ObstacleAvoidanceSensors from '../../common/components/ObstacleAvoidanceSensors';
import { CAUTION_DISTANCE, obstacleColor } from '../../common/util/obstacles';
import { contrastText } from '../../common/theme/palette';
import { devicesActions, sessionActions } from '../../store';
import { blink } from './animations';

// three.js is heavy: only download the 3D scene when it is first shown.
const DeviceScene3D = lazy(() => import('./DeviceScene3D'));

const EASE = 'all .35s cubic-bezier(.3,.7,.2,1)';
const HUD_COLOR = '#f2f2f3';

const useStyles = makeStyles()((theme) => ({
  root: {
    display: 'flex',
    flexDirection: 'column',
    minHeight: 0,
    height: '100%',
  },
  bar: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1),
    padding: theme.spacing(1, 1.5),
    borderBottom: `1px solid ${theme.palette.divider}`,
    whiteSpace: 'nowrap',
  },
  spacer: {
    flex: 1,
  },
  alert: {
    display: 'flex',
    alignItems: 'center',
    gap: 6,
    padding: '3px 10px',
    fontSize: 12,
    fontWeight: 600,
  },
  alertDot: {
    width: 7,
    height: 7,
    borderRadius: '50%',
    background: 'currentColor',
    animation: `${blink} 1s infinite`,
  },
  stage: {
    position: 'relative',
    flex: 1,
    minHeight: 0,
    overflow: 'hidden',
    background: theme.palette.background.default,
  },
  view: {
    position: 'absolute',
    overflow: 'hidden',
    transition: EASE,
  },
  camera: {
    background: '#3a3c40',
    '& > *:first-of-type': { width: '100%', height: '100%', objectFit: 'contain' },
  },
  pipLabel: {
    position: 'absolute',
    left: 0,
    right: 0,
    bottom: 0,
    padding: '2px 8px',
    fontSize: 11,
    color: HUD_COLOR,
    background: 'rgba(0,0,0,.55)',
    zIndex: 2,
  },
  pipCatcher: {
    position: 'absolute',
    inset: 0,
    zIndex: 2,
    cursor: 'pointer',
    border: 0,
    padding: 0,
    background: 'transparent',
  },
  hud: {
    position: 'absolute',
    inset: 0,
    pointerEvents: 'none',
    color: HUD_COLOR,
    fontVariantNumeric: 'tabular-nums',
    textShadow: '0 1px 2px rgba(0,0,0,.6)',
    zIndex: 1,
  },
  hudTop: {
    position: 'absolute',
    top: 10,
    left: 12,
    right: 12,
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1.5),
    fontSize: 12,
  },
  hudButton: {
    pointerEvents: 'auto',
    color: HUD_COLOR,
    background: 'rgba(0,0,0,.45)',
    '&:hover': { background: 'rgba(0,0,0,.65)' },
  },
  crosshair: {
    position: 'absolute',
    left: '50%',
    top: '50%',
    width: 70,
    height: 70,
    transform: 'translate(-50%, -50%)',
    '& > span': { position: 'absolute', background: HUD_COLOR },
  },
  readout: {
    position: 'absolute',
    top: 52,
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(1),
  },
  readoutLabel: {
    fontSize: 10,
    letterSpacing: '0.1em',
  },
  readoutValue: {
    fontSize: 20,
    fontWeight: 600,
    lineHeight: 1.1,
  },
  coords: {
    position: 'absolute',
    bottom: 12,
    left: '50%',
    transform: 'translateX(-50%)',
    fontSize: 12,
  },
  mapChips: {
    position: 'absolute',
    top: 10,
    left: 10,
    zIndex: 2,
    display: 'flex',
    gap: 6,
    pointerEvents: 'none',
  },
  chip: {
    padding: '2px 8px',
    fontSize: 12,
    background: theme.palette.background.paper,
    border: `1px solid ${theme.palette.divider}`,
    fontVariantNumeric: 'tabular-nums',
  },
  overlay: {
    position: 'absolute',
    left: 14,
    bottom: 14,
    zIndex: 5,
    width: 264,
    padding: theme.spacing(0.25, 1, 0.5, 1),
    opacity: 0.94,
    boxShadow: theme.shadows[4],
  },
  overlayHeader: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(0.5),
    marginBottom: theme.spacing(0.5),
  },
}));

const FULL = { left: 0, top: 0, width: '100%', height: '100%', zIndex: 1 };
const PIP = {
  left: 'calc(100% - 292px)',
  top: 'calc(100% - 182px)',
  width: 280,
  height: 170,
  zIndex: 3,
  borderWidth: 1,
  borderStyle: 'solid',
};
const HALF_LEFT = { left: 0, top: 0, width: 'calc(50% - 1px)', height: '100%', zIndex: 1 };
const HALF_RIGHT = {
  left: 'calc(50% + 1px)',
  top: 0,
  width: 'calc(50% - 1px)',
  height: '100%',
  zIndex: 1,
};

const geometryFor = (view) => {
  if (view === 'split') return { camera: HALF_LEFT, map: HALF_RIGHT };
  if (view === 'map') return { camera: PIP, map: FULL };
  return { camera: FULL, map: PIP };
};

// Equirectangular distance in meters; plenty for "is it inside the 3D ground".
const metersBetween = (a, b) =>
  Math.hypot(
    (b.lat - a.lat) * 111320,
    (b.lng - a.lng) * 111320 * Math.cos((a.lat * Math.PI) / 180),
  );

const fixed = (value, digits = 1) => (value == null ? '—' : Number(value).toFixed(digits));

/** Closest horizontal obstacle as [side, distance]. */
const nearest = (obstacle) =>
  ['front', 'back', 'left', 'right']
    .map((side) => [side, obstacle[side]])
    .sort((a, b) => a[1] - b[1])[0];

const CameraHud = ({ position, gimbalPitch, onSnapshot }) => {
  const { classes } = useStyles();
  const home = position?.attributes?.home;
  const altitude =
    home && position?.altitude != null ? position.altitude - home[2] : position?.altitude;
  return (
    <div className={classes.hud}>
      <div className={classes.hudTop}>
        {gimbalPitch != null && <span>{`GIMBAL ${Math.round(gimbalPitch)}°`}</span>}
        <span className={classes.spacer} />
        <IconButton
          size="small"
          className={classes.hudButton}
          onClick={onSnapshot}
          title="Snapshot"
        >
          <CameraAltIcon fontSize="small" />
        </IconButton>
      </div>
      <div className={classes.crosshair}>
        <span style={{ left: 0, top: 34, width: 24, height: 2 }} />
        <span style={{ right: 0, top: 34, width: 24, height: 2 }} />
        <span style={{ left: 34, top: 0, width: 2, height: 24 }} />
        <span style={{ left: 34, bottom: 0, width: 2, height: 24 }} />
      </div>
      <div className={classes.readout} style={{ left: 14 }}>
        <div>
          <div className={classes.readoutLabel}>{home ? 'ALT' : 'ALT AMSL'}</div>
          <div className={classes.readoutValue}>{`${fixed(altitude)} m`}</div>
        </div>
      </div>
      <div className={classes.readout} style={{ right: 14, textAlign: 'right' }}>
        <div>
          <div className={classes.readoutLabel}>SPEED</div>
          <div className={classes.readoutValue}>{`${fixed(position?.speed)} m/s`}</div>
        </div>
        <div>
          <div className={classes.readoutLabel}>HDG</div>
          <div
            className={classes.readoutValue}
          >{`${String(Math.round(position?.course ?? 0)).padStart(3, '0')}°`}</div>
        </div>
      </div>
      {position?.latitude != null && (
        <div
          className={classes.coords}
        >{`${fixed(position.latitude, 6)}, ${fixed(position.longitude, 6)}`}</div>
      )}
    </div>
  );
};

const DeviceStage = ({
  deviceId,
  position,
  capabilities,
  obstacle,
  view,
  onViewChange,
  obstacleOpen,
  onObstacleToggle,
  mapMode,
  onMapModeChange,
  mapProps,
  ghost,
  gimbalPitch,
}) => {
  const { classes, theme } = useStyles();
  const dispatch = useDispatch();
  const follow = useSelector((state) => state.devices.follow);
  const cameraRef = useRef(null);

  const hasCamera = capabilities.camera;
  const effectiveView = hasCamera ? view : 'map';
  const geometry = geometryFor(effectiveView);
  const cameraPip = hasCamera && effectiveView === 'map';
  const mapPip = hasCamera && effectiveView === 'cam';

  const close = capabilities.obstacle && obstacle ? nearest(obstacle) : null;
  const alert = close && close[1] < CAUTION_DISTANCE ? close : null;

  const home = position?.attributes?.home;
  const toHome =
    home && position?.latitude != null
      ? Math.round(
          Math.hypot(
            (position.latitude - home[0]) * 111320,
            (position.longitude - home[1]) * 111320 * Math.cos((position.latitude * Math.PI) / 180),
          ),
        )
      : null;

  const handleSnapshot = () => {
    const dataUrl = cameraRef.current?.capture();
    if (!dataUrl) return;
    const link = document.createElement('a');
    link.href = dataUrl;
    link.download = `snap_${cameraRef.current?.cameraSrc || deviceId}_${dayjs().format('YYYYMMDD_HHmmss')}.jpg`;
    link.click();
  };

  const is3d = mapMode === '3d';
  const mapName = is3d ? '3D' : 'Map';
  const title =
    effectiveView === 'map'
      ? mapName
      : effectiveView === 'split'
        ? `Camera + ${mapName}`
        : 'Camera';

  // The 3D scene is meant to be centered on the UAV: whenever it is shown (also when it comes
  // from the remembered mode on load), turn Follow on. The user can still turn it off.
  useEffect(() => {
    if (is3d) dispatch(devicesActions.updateFollow(true));
  }, [is3d, dispatch]);

  // The 3D ground is only built around the scene origin (scene3d.range meters). The origin is
  // global (last mission/Pegman, or the default) and SelectDevice3D only recenters when the
  // selection CHANGES, which never happens here: the UAV is selected before the scene mounts.
  // So move the origin onto the UAV when it is outside the ground.
  const sceneOrigin = useSelector((state) => state.session.scene3d.origin);
  const sceneRange = useSelector((state) => state.session.scene3d.range);
  const latitude = position?.latitude;
  const longitude = position?.longitude;
  useEffect(() => {
    if (!is3d || latitude == null) return;
    if (metersBetween(sceneOrigin, { lat: latitude, lng: longitude }) <= sceneRange / 2) return;
    // alt 400: same origin altitude SelectDevice3D uses.
    dispatch(sessionActions.updateScene3dOrigin({ lat: latitude, lng: longitude, alt: 400 }));
  }, [is3d, latitude, longitude, sceneOrigin, sceneRange, dispatch]);
  const pipBorder = theme.palette.text.primary;

  return (
    <Paper variant="outlined" square className={classes.root}>
      <div className={classes.bar}>
        <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
          {title}
        </Typography>
        {position?.latitude != null && (
          <Typography variant="caption" color="textSecondary">
            {`${fixed(position.latitude, 5)}, ${fixed(position.longitude, 5)}`}
          </Typography>
        )}
        <span className={classes.spacer} />
        {alert && (
          <span
            className={classes.alert}
            style={{
              background: obstacleColor(alert[1]),
              color: contrastText(obstacleColor(alert[1])),
            }}
          >
            <span className={classes.alertDot} />
            {`Obstacle ${alert[1].toFixed(1)} m ${alert[0]}`}
          </span>
        )}
        <Button
          size="small"
          variant={follow ? 'contained' : 'outlined'}
          startIcon={<GpsFixedIcon />}
          onClick={() => dispatch(devicesActions.updateFollow(!follow))}
        >
          Follow
        </Button>
        {capabilities.obstacle && (
          <Button
            size="small"
            variant={obstacleOpen ? 'contained' : 'outlined'}
            startIcon={<RadarIcon />}
            onClick={onObstacleToggle}
          >
            Collision sensors
          </Button>
        )}
        <ToggleButtonGroup
          size="small"
          exclusive
          value={mapMode}
          onChange={(_, value) => value && onMapModeChange(value)}
          sx={{ height: 28 }}
        >
          <ToggleButton value="2d">2D</ToggleButton>
          <ToggleButton value="3d">3D</ToggleButton>
        </ToggleButtonGroup>
        {hasCamera && (
          <ToggleButtonGroup
            size="small"
            exclusive
            value={effectiveView}
            onChange={(_, value) => value && onViewChange(value)}
            sx={{ height: 28 }}
          >
            <ToggleButton value="cam">Camera</ToggleButton>
            <ToggleButton value="map">Map</ToggleButton>
            <ToggleButton value="split">Split</ToggleButton>
          </ToggleButtonGroup>
        )}
      </div>
      <div className={classes.stage}>
        <div
          className={classes.view}
          style={{
            ...geometry.map,
            borderColor: pipBorder,
            boxShadow: mapPip ? theme.shadows[8] : 'none',
          }}
        >
          {is3d ? (
            <Suspense fallback={null}>
              <DeviceScene3D ghost={ghost} />
            </Suspense>
          ) : (
            <MainMap {...mapProps}>
              {ghost && (
                <MapGhostTarget
                  from={ghost.from}
                  to={ghost.to}
                  course={ghost.course}
                  label={ghost.label}
                />
              )}
            </MainMap>
          )}
          {!mapPip && (
            <div className={classes.mapChips}>
              {position?.latitude != null && (
                <span
                  className={classes.chip}
                >{`${fixed(position.latitude, 6)}, ${fixed(position.longitude, 6)}`}</span>
              )}
              {toHome != null && <span className={classes.chip}>{`${toHome} m to home`}</span>}
            </div>
          )}
          {mapPip && (
            <>
              <button
                type="button"
                className={classes.pipCatcher}
                aria-label={`Expand ${mapName}`}
                onClick={() => onViewChange('map')}
              />
              <div className={classes.pipLabel}>{`${mapName} · click to expand`}</div>
            </>
          )}
        </div>
        {hasCamera && (
          <div
            className={`${classes.view} ${classes.camera}`}
            style={{
              ...geometry.camera,
              borderColor: pipBorder,
              boxShadow: cameraPip ? theme.shadows[8] : 'none',
            }}
          >
            <CameraDevice ref={cameraRef} deviceId={deviceId} />
            {cameraPip ? (
              <>
                <button
                  type="button"
                  className={classes.pipCatcher}
                  aria-label="Expand camera"
                  onClick={() => onViewChange('cam')}
                />
                <div className={classes.pipLabel}>Camera · click to expand</div>
              </>
            ) : (
              <CameraHud
                position={position}
                gimbalPitch={gimbalPitch}
                onSnapshot={handleSnapshot}
              />
            )}
          </div>
        )}
        {capabilities.obstacle && obstacleOpen && obstacle && (
          <Paper variant="outlined" square className={classes.overlay}>
            <div className={classes.overlayHeader}>
              <Typography variant="subtitle2">Collision sensors</Typography>
              <Typography variant="caption" color="textSecondary">
                obstacle_info
              </Typography>
              <span className={classes.spacer} />
              <IconButton
                size="small"
                onClick={onObstacleToggle}
                aria-label="Close collision sensors"
              >
                <CloseIcon fontSize="small" />
              </IconButton>
            </div>
            <ObstacleAvoidanceSensors obstacleData={obstacle} />
          </Paper>
        )}
      </div>
    </Paper>
  );
};

export default DeviceStage;
