import { useEffect, useState } from 'react';
import { useDispatch, useSelector, shallowEqual } from 'react-redux';
import { useNavigate, useParams } from 'react-router-dom';
import { Alert, Snackbar } from '@mui/material';
import { makeStyles } from 'tss-react/mui';

import usePersistedState from '../common/util/usePersistedState';
import useFilter from '../components/devices/useFilter';
import CommandCard from '../components/commands/CommandCard';
import { CommandType } from '../common/util/commandTypes';
import { obstacleFromInfo } from '../common/util/obstacles';
import { ARM_STATE, armLabel } from '../common/util/vehicleState';
import { devicesActions } from '../store';

import DeviceHeader from './device/DeviceHeader';
import DeviceStage from './device/DeviceStage';
import MissionControlCard from './device/MissionControlCard';
import TelemetryGrid from './device/TelemetryGrid';
import TelemetryLogPanel from './device/TelemetryLogPanel';
import ManualControlPanel from './device/ManualControlPanel';
import ConfirmDialog from './device/ConfirmDialog';
import { useDeviceCapabilities, selectDeviceMission } from './device/useDeviceCapabilities';
import { useDeviceCommands } from './device/useDeviceCommands';
import { useTelemetryLog } from './device/useTelemetryLog';
import { useControlKeys } from './device/useControlKeys';
import { useMissionActions } from './device/useMissionActions';
import {
  STEPS,
  EMPTY_PENDING,
  nudge,
  ghostTarget,
  hasPending,
  hasPendingMove,
  hasPendingGimbal,
} from './device/manualControl';

const useStyles = makeStyles()((theme) => ({
  root: {
    height: '100vh',
    minWidth: 1280,
    minHeight: 780,
    display: 'grid',
    gridTemplateRows: '62px minmax(0, 1fr)',
    background: theme.palette.background.default,
    fontVariantNumeric: 'tabular-nums',
  },
  main: {
    display: 'grid',
    gap: 14,
    padding: '14px 20px 18px',
    minHeight: 0,
  },
  area: {
    minWidth: 0,
    minHeight: 0,
  },
}));

// Layout "A · Stage + rail" of the design: stage on the left, mission/telemetry/telemetry log on a
// 400px rail, manual control under the stage. Areas of capabilities the device lacks are dropped.
const gridFor = ({ mission, control }) => {
  const rail = [...(mission ? ['mission'] : []), 'telem', 'raw'];
  const areas = rail.map((area) => `"stage ${area}"`);
  const rows = rail.map((area) => (area === 'raw' ? 'minmax(0, 1fr)' : 'max-content'));
  if (control) {
    areas.push('"ctrl raw"');
    rows.push('262px');
  }
  return {
    gridTemplateColumns: 'minmax(0, 1fr) 400px',
    gridTemplateRows: rows.join(' '),
    gridTemplateAreas: areas.join(' '),
  };
};

const QUICK_COMMANDS = [
  CommandType.TAKEOFF,
  CommandType.LAND,
  CommandType.RETURN_HOME,
  CommandType.HOLD,
  CommandType.ARM,
  CommandType.DISARM,
  CommandType.KILL_MOTORS,
];

// Quick commands the device supports; with a known arm state only Arm or Disarm, not both.
const availableQuickCommands = (capabilities, armed) =>
  QUICK_COMMANDS.filter((type) => capabilities.has(type))
    .filter(
      (type) =>
        armed == null || (type === CommandType.ARM ? !armed : type !== CommandType.DISARM || armed),
    )
    .map((type) => ({
      type,
      label: capabilities.label(type),
      danger: capabilities.command(type)?.severity === 'danger',
    }));

const DevicePage = () => {
  const { classes } = useStyles();
  const navigate = useNavigate();
  const dispatch = useDispatch();
  const { id } = useParams();

  const positions = useSelector((state) => state.session.positions);
  const device = useSelector((state) => state.devices.items[id]);
  const markers = useSelector((state) => state.session.markers);
  const routes = useSelector((state) => state.mission.route);
  const missionInfo = useSelector(selectDeviceMission(id), shallowEqual);

  const position = positions[id];
  const attributes = position?.attributes ?? {};

  const [filter] = usePersistedState('filter', {
    statuses: [],
    vehicleTypes: [],
    groups: [],
  });
  const [filterSort] = usePersistedState('filterSort', '');
  const [filterMap] = usePersistedState('filterMap', false);
  const { filteredPositions } = useFilter('', filter, filterSort, filterMap, positions);

  const capabilities = useDeviceCapabilities(device);
  const stream = useTelemetryLog(position);
  const { send, toast, closeToast, notify } = useDeviceCommands(
    id,
    position,
    stream.log,
    capabilities.label,
  );

  const [view, setView] = useState('cam');
  const [mapMode, setMapMode] = usePersistedState('deviceMapMode', '2d');
  const [obstacleOpen, setObstacleOpen] = useState(true);
  const [commandOpen, setCommandOpen] = useState(false);
  const [confirmation, setConfirmation] = useState(null);
  const [locked, setLocked] = useState(true);
  const [step, setStep] = useState(1);
  const [pending, setPending] = useState(EMPTY_PENDING);

  useEffect(() => {
    if (id != null) dispatch(devicesActions.selectId(parseInt(id, 10)));
  }, [dispatch, id]);

  // attributes.gimbal = [pitch, roll, yaw] in degrees (same reading as RouteRouteList).
  const gimbal = {
    pitch: Math.round(attributes.gimbal?.[0] ?? 0),
    yaw: Math.round(attributes.gimbal?.[2] ?? 0),
  };
  const armed = armLabel(attributes.armState) ? attributes.armState === ARM_STATE.ARMED : null;
  const obstacle = capabilities.obstacle ? obstacleFromInfo(attributes.obstacle_info) : null;

  const askConfirm = (options) => setConfirmation(options);

  // A command whose schema has `confirm` asks first; its message is a `{device}` template.
  const sendQuick = (type) => {
    const label = capabilities.label(type);
    const confirm = capabilities.command(type)?.confirm;
    if (!confirm) {
      send(type);
      return;
    }
    askConfirm({
      title: `${label}?`,
      message: confirm.message.replaceAll('{device}', device?.name ?? 'The device'),
      confirmLabel: label,
      onConfirm: () => send(type),
    });
  };

  const mission = useMissionActions({
    missionInfo,
    capabilities,
    send,
    askConfirm,
    deviceName: device?.name,
  });

  const toggleLock = async () => {
    setPending(EMPTY_PENDING);
    if (!locked) {
      setLocked(true);
      notify('Control released');
      return;
    }
    setLocked(false);
    if (!(await mission.pauseForControl())) notify('You have control');
  };

  const applyPending = async () => {
    if (!hasPending(pending)) return;
    const p = pending;
    setPending(EMPTY_PENDING);
    if (hasPendingMove(p)) {
      await send(
        CommandType.MOVE_RELATIVE,
        { dx: p.fwd, dy: p.right, dz: p.up, dyaw: p.yaw },
        'Relative move',
      );
    }
    if (hasPendingGimbal(p)) {
      await send(
        CommandType.GIMBAL,
        { pitch: gimbal.pitch + p.gp, yaw: gimbal.yaw + p.gy },
        'Gimbal move',
      );
    }
  };

  const nudgePending = (delta) =>
    setPending((current) => nudge(current, delta, STEPS[step], gimbal.pitch));

  // Keyboard only with control taken and no dialog open.
  useControlKeys({
    enabled: !locked && !commandOpen && !confirmation && capabilities.control,
    gimbal: capabilities.gimbal,
    onNudge: nudgePending,
    onApply: applyPending,
    onClear: () => setPending(EMPTY_PENDING),
  });

  const ghost = ghostTarget(position, pending);

  const quickCommands = availableQuickCommands(capabilities, armed);

  const showControl = capabilities.control || capabilities.gimbal || quickCommands.length > 0;

  return (
    <div className={classes.root}>
      <DeviceHeader
        device={device}
        position={position}
        capabilities={capabilities}
        inMission={Boolean(missionInfo)}
        onBack={() => navigate('/')}
        onSendCommand={() => setCommandOpen(true)}
        onOpenMessages={() => navigate(`/device/${id}/messages`)}
      />
      <main
        className={classes.main}
        style={gridFor({ mission: missionInfo, control: showControl })}
      >
        <div className={classes.area} style={{ gridArea: 'stage' }}>
          <DeviceStage
            deviceId={device?.id}
            position={position}
            capabilities={capabilities}
            obstacle={obstacle}
            view={view}
            onViewChange={setView}
            mapMode={mapMode}
            onMapModeChange={setMapMode}
            obstacleOpen={obstacleOpen}
            onObstacleToggle={() => setObstacleOpen(!obstacleOpen)}
            mapProps={{
              filteredPositions,
              markers,
              routes,
              selectedPosition: id,
              filteredMissiondeviceid: id,
            }}
            ghost={ghost}
            gimbalPitch={capabilities.gimbal ? gimbal.pitch : null}
          />
        </div>
        {missionInfo && (
          <div className={classes.area} style={{ gridArea: 'mission' }}>
            <MissionControlCard
              mission={missionInfo.mission}
              task={missionInfo.task}
              paused={mission.paused}
              actions={mission.actions}
            />
          </div>
        )}
        <div className={classes.area} style={{ gridArea: 'telem' }}>
          <TelemetryGrid position={position} />
        </div>
        <div className={classes.area} style={{ gridArea: 'raw' }}>
          <TelemetryLogPanel position={position} stream={stream} />
        </div>
        {showControl && (
          <div className={classes.area} style={{ gridArea: 'ctrl' }}>
            <ManualControlPanel
              deviceName={device?.name}
              locked={locked}
              onToggleLock={toggleLock}
              armed={armed}
              canMove={capabilities.control}
              hasGimbal={capabilities.gimbal}
              gimbal={gimbal}
              step={step}
              onStepChange={setStep}
              pending={pending}
              onNudge={nudgePending}
              onCenterGimbal={() => setPending({ ...pending, gp: -gimbal.pitch, gy: -gimbal.yaw })}
              onClear={() => setPending(EMPTY_PENDING)}
              onApply={applyPending}
              quickCommands={quickCommands}
              onQuickCommand={sendQuick}
            />
          </div>
        )}
      </main>

      {commandOpen && (
        <CommandCard
          id={id}
          onClose={() => setCommandOpen(false)}
          onSend={(type, commandAttributes) => send(type, commandAttributes)}
        />
      )}
      <ConfirmDialog
        open={Boolean(confirmation)}
        title={confirmation?.title}
        message={confirmation?.message}
        cancelLabel={confirmation?.cancelLabel}
        confirmLabel={confirmation?.confirmLabel}
        onCancel={() => setConfirmation(null)}
        onConfirm={() => {
          confirmation.onConfirm();
          setConfirmation(null);
        }}
      />
      <Snackbar
        key={toast?.key}
        open={Boolean(toast)}
        autoHideDuration={2600}
        onClose={closeToast}
        anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
      >
        {toast ? (
          <Alert severity={toast.severity} variant="filled" onClose={closeToast}>
            {toast.message}
          </Alert>
        ) : undefined}
      </Snackbar>
    </div>
  );
};

export default DevicePage;
