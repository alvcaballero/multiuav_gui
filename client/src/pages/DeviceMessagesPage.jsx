import { useState } from 'react';
import { useSelector } from 'react-redux';
import { useNavigate, useParams } from 'react-router-dom';
import { Button, List, ListItemButton, ListItemText, Paper, Typography } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import dayjs from 'dayjs';

import JsonTree from './deviceMessages/JsonTree';
import { useCategorySubscribers } from './deviceMessages/useCategorySubscribers';
import { useRawMessage } from './deviceMessages/useRawMessage';
import { useSlotTelemetry } from './deviceMessages/useSlotTelemetry';

const useStyles = makeStyles()((theme) => ({
  root: {
    height: '100vh',
    minWidth: 960,
    display: 'grid',
    gridTemplateRows: '62px minmax(0, 1fr)',
    background: theme.palette.background.default,
  },
  header: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(2),
    padding: theme.spacing(0, 2.5),
    borderBottom: `1px solid ${theme.palette.divider}`,
    whiteSpace: 'nowrap',
  },
  name: {
    display: 'flex',
    alignItems: 'baseline',
    gap: theme.spacing(1),
  },
  spacer: { flex: 1 },
  main: {
    display: 'grid',
    gridTemplateColumns: '340px minmax(0, 1fr)',
    gap: theme.spacing(2),
    padding: theme.spacing(2, 2.5),
    minHeight: 0,
  },
  panel: {
    display: 'flex',
    flexDirection: 'column',
    minHeight: 0,
  },
  panelHeader: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1),
    padding: theme.spacing(1, 1.5),
    borderBottom: `1px solid ${theme.palette.divider}`,
    minHeight: 44,
  },
  scroll: {
    flex: 1,
    minHeight: 0,
    overflow: 'auto',
  },
  body: {
    flex: 1,
    minHeight: 0,
    overflow: 'auto',
    padding: theme.spacing(1.5),
  },
  notice: {
    padding: theme.spacing(3),
    color: theme.palette.text.secondary,
  },
  mono: {
    fontFamily: 'ui-monospace, Menlo, monospace',
    fontSize: 12,
  },
}));

const REJECTED = {
  not_inspectable: 'This message cannot be inspected here (the camera has its own channel).',
  invalid: 'The server did not understand the request.',
  too_many: 'Too many messages are being inspected from this window.',
};

const formatAge = (ms) => (ms < 1000 ? `${ms} ms` : `${(ms / 1000).toFixed(1)} s`);

// What the watchdog says of one slot: its rate, or why there is none.
const SlotRate = ({ telemetry }) => {
  const { classes } = useStyles();
  if (telemetry?.health === 'missing') {
    return (
      <Typography variant="caption" color="warning.main" className={classes.mono}>
        never arrived
      </Typography>
    );
  }
  if (telemetry?.health === 'stale') {
    return (
      <Typography variant="caption" color="warning.main" className={classes.mono}>
        stopped {formatAge(telemetry.ageMs)} ago
      </Typography>
    );
  }
  return (
    <Typography
      variant="caption"
      color="textSecondary"
      className={classes.mono}
      title={telemetry?.ageMs != null ? `last one ${formatAge(telemetry.ageMs)} ago` : undefined}
    >
      {telemetry?.hz != null ? `${telemetry.hz.toFixed(1)} Hz` : '—'}
    </Typography>
  );
};

const Notice = ({ children }) => {
  const { classes } = useStyles();
  return <div className={classes.notice}>{children}</div>;
};

const Viewer = ({ slot, entry, raw }) => {
  if (!raw.connected)
    return <Notice>The WebSocket is disconnected. Reload the page once it is back.</Notice>;
  if (raw.status === 'idle') return <Notice>Pick a message from the list.</Notice>;
  if (raw.status === 'rejected')
    return <Notice>{REJECTED[raw.reason] ?? `Rejected (${raw.reason}).`}</Notice>;
  if (raw.status === 'waiting') {
    return (
      <Notice>
        Waiting for <b>{slot}</b>
        {entry?.name ? ` (${entry.name})` : ''}. If the device is not sending it, nothing will show
        up.
      </Notice>
    );
  }
  return <JsonTree value={raw.message.data} />;
};

const DeviceMessagesPage = () => {
  const { classes } = useStyles();
  const navigate = useNavigate();
  const { id } = useParams();

  const device = useSelector((state) => state.devices.items[id]);
  const subscribers = useCategorySubscribers(device?.category);
  const [slot, setSlot] = useState(null);
  const raw = useRawMessage(id, slot);
  const telemetry = useSlotTelemetry(id);

  // The camera is binary and has its own channel; the server would refuse it anyway.
  const slots = Object.entries(subscribers ?? {}).filter(([name]) => name !== 'camera');
  const message = raw.status === 'streaming' ? raw.message : null;

  const copy = () => navigator.clipboard?.writeText(JSON.stringify(message.data, null, 2));

  return (
    <div className={classes.root}>
      <header className={classes.header}>
        <Button
          size="small"
          color="inherit"
          startIcon={<ArrowBackIcon />}
          onClick={() => navigate(`/device/${id}`)}
        >
          Device
        </Button>
        <div className={classes.name}>
          <Typography variant="h6" component="h1">
            {device?.name ?? 'Unknown device'}
          </Typography>
          <Typography variant="caption" color="textSecondary">
            {[device?.category, device?.protocol].filter(Boolean).join(' · ')}
          </Typography>
        </div>
        <div className={classes.spacer} />
        <Typography variant="caption" color={raw.connected ? 'success.main' : 'error.main'}>
          {raw.connected ? 'WebSocket connected' : 'WebSocket disconnected'}
        </Typography>
      </header>

      <main className={classes.main}>
        <Paper variant="outlined" square className={classes.panel}>
          <div className={classes.panelHeader}>
            <Typography variant="overline" color="primary" sx={{ lineHeight: 1.2 }}>
              Messages
            </Typography>
          </div>
          <div className={classes.scroll}>
            {subscribers === null ? (
              <Notice>Loading…</Notice>
            ) : slots.length === 0 ? (
              <Notice>This category declares no messages.</Notice>
            ) : (
              <List dense disablePadding>
                {slots.map(([name, entry]) => (
                  <ListItemButton key={name} selected={name === slot} onClick={() => setSlot(name)}>
                    <ListItemText
                      primary={name}
                      secondary={[entry.name, entry.messageType].filter(Boolean).join(' · ')}
                      slotProps={{ secondary: { className: classes.mono, noWrap: true } }}
                    />
                    <SlotRate telemetry={telemetry[name]} />
                  </ListItemButton>
                ))}
              </List>
            )}
          </div>
        </Paper>

        <Paper variant="outlined" square className={classes.panel}>
          <div className={classes.panelHeader}>
            <Typography variant="overline" color="primary" sx={{ lineHeight: 1.2 }}>
              {slot ?? 'Message'}
            </Typography>
            {message && (
              <Typography variant="caption" color="textSecondary" className={classes.mono}>
                {message.transport} ·{' '}
                {message.cached
                  ? `stored, received ${formatAge(message.ageMs)} before opening`
                  : dayjs(raw.receivedAt).format('HH:mm:ss.SSS')}
              </Typography>
            )}
            <div className={classes.spacer} />
            <Button size="small" variant="outlined" disabled={!message} onClick={raw.togglePaused}>
              {raw.paused ? 'Resume' : 'Pause'}
            </Button>
            <Button size="small" variant="outlined" disabled={!message} onClick={copy}>
              Copy
            </Button>
          </div>
          <div className={classes.body}>
            <Viewer slot={slot} entry={subscribers?.[slot]} raw={raw} />
          </div>
        </Paper>
      </main>
    </div>
  );
};

export default DeviceMessagesPage;
