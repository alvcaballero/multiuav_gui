import { Paper, Typography } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import dayjs from 'dayjs';
import {
  armLabel,
  isAbnormalStop,
  missionEventLabel,
  missionExecLabel,
  phaseLabel,
} from '../../common/util/vehicleState';

const useStyles = makeStyles()((theme) => ({
  root: {
    padding: theme.spacing(1.5, 1.75),
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(1),
    fontVariantNumeric: 'tabular-nums',
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  grid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(auto-fill, minmax(128px, 1fr))',
    borderTop: `1px solid ${theme.palette.divider}`,
    borderLeft: `1px solid ${theme.palette.divider}`,
  },
  cell: {
    padding: theme.spacing(0.75, 1),
    borderRight: `1px solid ${theme.palette.divider}`,
    borderBottom: `1px solid ${theme.palette.divider}`,
  },
  label: {
    fontSize: 10,
    letterSpacing: '0.1em',
    textTransform: 'uppercase',
    color: theme.palette.text.secondary,
  },
  value: {
    fontSize: 23,
    fontWeight: 600,
    lineHeight: 1.15,
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
  unit: {
    fontSize: 11.5,
    fontWeight: 400,
    marginLeft: 3,
    color: theme.palette.text.secondary,
  },
  sub: {
    fontSize: 11,
    color: theme.palette.text.secondary,
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
  events: {
    display: 'grid',
    gridTemplateColumns: 'max-content minmax(0, 1fr)',
    columnGap: theme.spacing(1),
    rowGap: 2,
    fontSize: 12,
  },
  eventRow: {
    display: 'contents',
  },
  eventTime: {
    color: theme.palette.text.secondary,
  },
  eventText: {
    whiteSpace: 'nowrap',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
}));

// The newest of `attributes.missionEvents` shown under the grid (the server keeps a few more).
const EVENTS_SHOWN = 4;

const fixed = (value, digits = 1) =>
  value == null || Number.isNaN(Number(value)) ? '—' : Number(value).toFixed(digits);

const toRad = (deg) => (deg * Math.PI) / 180;

/** Distance (m, equirectangular) and bearing (deg) from `p` to `home` [lat, lon]. */
const homeVector = (p, home) => {
  const dNorth = (home[0] - p.latitude) * 111320;
  const dEast = (home[1] - p.longitude) * 111320 * Math.cos(toRad(p.latitude));
  const bearing = ((Math.atan2(dEast, dNorth) * 180) / Math.PI + 360) % 360;
  return { distance: Math.hypot(dNorth, dEast), bearing };
};

const batteryColor = (theme, level) =>
  level < 15 ? theme.palette.error.main : level < 30 ? theme.palette.warning.main : undefined;

// Only what the GCS actually receives: cells for data the device doesn't publish show "—".
const TelemetryGrid = ({ position }) => {
  const { classes, theme } = useStyles();
  const a = position?.attributes ?? {};
  const home = a.home;
  const relAltitude = home && position?.altitude != null ? position.altitude - home[2] : null;
  const toHome = home && position?.latitude != null ? homeVector(position, home) : null;
  const gimbal = Array.isArray(a.gimbal) ? a.gimbal : null;
  // What the vehicle says about the mission it holds (DJI WaypointV2), when it says it: it wins over
  // the GCS-side mission_state, which a PSDK vehicle never reports.
  const missionExec = missionExecLabel(a.missionExecState);
  const missionEvents = (a.missionEvents ?? []).slice(0, EVENTS_SHOWN);

  const cells = [
    {
      label: 'Altitude',
      value: fixed(relAltitude ?? position?.altitude),
      unit: 'm',
      sub: relAltitude != null ? `AMSL ${fixed(position.altitude, 0)} m` : 'AMSL (no home set)',
    },
    { label: 'Ground speed', value: fixed(position?.speed), unit: 'm/s', sub: '' },
    {
      label: 'Heading',
      value: position?.course != null ? String(Math.round(position.course)).padStart(3, '0') : '—',
      unit: '°',
      sub: '',
    },
    {
      label: 'Battery',
      value: fixed(a.batteryLevel, 0),
      unit: '%',
      sub: '',
      color: a.batteryLevel != null ? batteryColor(theme, a.batteryLevel) : undefined,
    },
    {
      label: 'To home',
      value: toHome ? fixed(toHome.distance, 0) : '—',
      unit: 'm',
      sub: toHome ? `bearing ${Math.round(toHome.bearing)}°` : 'no home set',
    },
    {
      label: 'Gimbal',
      value: gimbal ? fixed(gimbal[0], 0) : '—',
      unit: '°',
      sub: gimbal ? `pitch · yaw ${fixed(gimbal[2], 0)}°` : '',
    },
    { label: 'Flight', value: phaseLabel(a.flightPhase) ?? '—', sub: armLabel(a.armState) ?? '' },
    missionExec
      ? {
          label: 'Mission',
          value: missionExec,
          sub: `wp ${a.missionExecWp ?? '—'} · ${fixed(a.missionExecSpeed)} m/s`,
        }
      : {
          label: 'Mission',
          value: a.mission_state ?? '—',
          sub: a.wp_reached != null ? `wp ${a.wp_reached}` : '',
        },
  ];

  return (
    <Paper variant="outlined" square className={classes.root}>
      <div className={classes.header}>
        <Typography variant="overline" color="primary" sx={{ lineHeight: 1.2 }}>
          Telemetry
        </Typography>
        <Typography variant="caption" color="textSecondary">
          {position?.deviceTime ? dayjs(position.deviceTime).format('HH:mm:ss') : '—'}
        </Typography>
      </div>
      <div className={classes.grid}>
        {cells.map(({ label, value, unit, sub, color }) => (
          <div key={label} className={classes.cell}>
            <div className={classes.label}>{label}</div>
            <div className={classes.value} style={{ color }}>
              {value}
              {unit && value !== '—' && <span className={classes.unit}>{unit}</span>}
            </div>
            <div className={classes.sub}>{sub || ' '}</div>
          </div>
        ))}
      </div>
      {missionEvents.length > 0 && (
        <div className={classes.events}>
          {missionEvents.map((event) => (
            <div key={`${event.raw}-${event.stamp}-${event.at}`} className={classes.eventRow}>
              <span className={classes.eventTime}>{dayjs(event.at).format('HH:mm:ss')}</span>
              <span
                className={classes.eventText}
                style={
                  isAbnormalStop(event)
                    ? { color: theme.palette.error.main, fontWeight: 600 }
                    : undefined
                }
              >
                {missionEventLabel(event)}
              </span>
            </div>
          ))}
        </div>
      )}
    </Paper>
  );
};

export default TelemetryGrid;
