import { Button, Typography } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import dayjs from 'dayjs';
import {
  FLIGHT_MODE,
  FLIGHT_PHASE,
  armLabel,
  modeLabel,
  phaseLabel,
  vtolLabel,
} from '../../common/util/vehicleState';
import { blink } from './animations';

const useStyles = makeStyles()((theme) => ({
  root: {
    height: 62,
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(2),
    padding: theme.spacing(0, 2.5),
    borderBottom: `1px solid ${theme.palette.divider}`,
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    fontVariantNumeric: 'tabular-nums',
  },
  divider: {
    width: 1,
    height: 28,
    background: theme.palette.divider,
    flex: 'none',
  },
  name: {
    display: 'flex',
    alignItems: 'baseline',
    gap: theme.spacing(1),
    flex: 'none',
  },
  pill: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1),
    padding: '4px 10px',
    border: '1px solid',
    flex: 'none',
  },
  dot: {
    width: 7,
    height: 7,
    borderRadius: '50%',
    animation: `${blink} 1.6s infinite`,
  },
  status: {
    fontWeight: 600,
    fontSize: 13,
    letterSpacing: '0.06em',
    textTransform: 'uppercase',
  },
  stats: {
    display: 'flex',
    gap: theme.spacing(2.25),
    flex: 'none',
  },
  statLabel: {
    fontSize: 10,
    letterSpacing: '0.1em',
    textTransform: 'uppercase',
    color: theme.palette.text.secondary,
  },
  statValue: {
    fontSize: 13.5,
    fontWeight: 500,
  },
  spacer: {
    flex: 1,
  },
  tags: {
    display: 'flex',
    gap: theme.spacing(0.75),
    flex: 'none',
  },
  tag: {
    fontSize: 11,
    padding: '2px 8px',
    border: '1px solid',
  },
}));

// Status shown in the pill, from the flight mode / phase when there is one.
const deviceStatus = (device, attributes = {}) => {
  if (attributes.flightMode === FLIGHT_MODE.RTL) return { text: 'Returning', color: 'error' };
  if (attributes.flightPhase === FLIGHT_PHASE.IN_AIR)
    return { text: 'In flight', color: 'success' };
  if (device?.status === 'online') return { text: 'Online', color: 'success' };
  return { text: device?.status ?? 'Unknown', color: 'neutral' };
};

const batteryColor = (level) => (level < 15 ? 'error' : level < 30 ? 'warning' : null);

const DeviceHeader = ({
  device,
  position,
  capabilities,
  inMission,
  onBack,
  onSendCommand,
  onOpenMessages,
}) => {
  const { classes, theme } = useStyles();
  const attributes = position?.attributes ?? {};
  const status = deviceStatus(device, attributes);
  const statusColor = theme.palette[status.color]?.main ?? theme.palette.text.disabled;
  const battery = attributes.batteryLevel;
  const battColor = battery != null ? batteryColor(battery) : null;

  const stats = [
    {
      label: 'Battery',
      value: battery != null ? `${battery}%` : '—',
      color: battColor ? theme.palette[battColor].main : undefined,
    },
    { label: 'Armed', value: armLabel(attributes.armState) ?? '—' },
    { label: 'Phase', value: phaseLabel(attributes.flightPhase) ?? '—' },
    // Only a VTOL reports it (MC / FW / transitioning): no stat at all for any other vehicle.
    ...(vtolLabel(attributes.vtolState)
      ? [{ label: 'VTOL', value: vtolLabel(attributes.vtolState) }]
      : []),
    {
      label: 'Last update',
      value: position?.deviceTime ? dayjs(position.deviceTime).format('HH:mm:ss') : '—',
    },
  ];

  const tags = [
    ['Camera', capabilities.camera],
    ['Gimbal', capabilities.gimbal],
    ['Collision', capabilities.obstacle],
    ['Control', capabilities.control],
    ['Mission', inMission],
  ];

  return (
    <header className={classes.root}>
      <Button size="small" color="inherit" startIcon={<ArrowBackIcon />} onClick={onBack}>
        Fleet
      </Button>
      <div className={classes.divider} />
      <div className={classes.name}>
        <Typography variant="h6" component="h1">
          {device?.name ?? 'Unknown device'}
        </Typography>
        <Typography variant="caption" color="textSecondary">
          {[device?.category, device?.protocol].filter(Boolean).join(' · ')}
        </Typography>
      </div>
      <div className={classes.pill} style={{ borderColor: statusColor }}>
        <span className={classes.dot} style={{ background: statusColor }} />
        <span className={classes.status} style={{ color: statusColor }}>
          {status.text}
        </span>
        {modeLabel(attributes.flightMode) && (
          <Typography variant="caption" color="textSecondary">
            {modeLabel(attributes.flightMode)}
          </Typography>
        )}
      </div>
      <div className={classes.stats}>
        {stats.map(({ label, value, color }) => (
          <div key={label}>
            <div className={classes.statLabel}>{label}</div>
            <div className={classes.statValue} style={{ color }}>
              {value}
            </div>
          </div>
        ))}
      </div>
      <div className={classes.spacer} />
      <div className={classes.tags}>
        {tags.map(([label, on]) => (
          <span
            key={label}
            className={classes.tag}
            style={
              on
                ? {
                    background: theme.palette.primary.main,
                    borderColor: theme.palette.primary.main,
                    color: theme.palette.primary.contrastText,
                  }
                : { borderColor: theme.palette.divider, color: theme.palette.text.disabled }
            }
          >
            {label}
          </span>
        ))}
      </div>
      <Button variant="outlined" onClick={onOpenMessages}>
        Messages
      </Button>
      <Button variant="contained" onClick={onSendCommand}>
        Send command
      </Button>
    </header>
  );
};

export default DeviceHeader;
