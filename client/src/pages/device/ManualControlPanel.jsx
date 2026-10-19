import { Button, Chip, Paper, ToggleButton, ToggleButtonGroup, Typography } from '@mui/material';
import { alpha } from '@mui/material/styles';
import { makeStyles } from 'tss-react/mui';

import { STEPS, formatSigned, pendingChips, hasPending } from './manualControl';

const PAD = 38;

const useStyles = makeStyles()((theme) => ({
  root: {
    position: 'relative',
    display: 'flex',
    flexDirection: 'column',
    minHeight: 0,
    height: '100%',
  },
  header: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1.25),
    padding: theme.spacing(1, 1.5),
    borderBottom: `1px solid ${theme.palette.divider}`,
    whiteSpace: 'nowrap',
  },
  hint: {
    flex: 1,
    minWidth: 0,
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  },
  body: {
    flex: 1,
    minHeight: 0,
    overflow: 'auto',
    display: 'flex',
    flexWrap: 'wrap',
    alignContent: 'flex-start',
    gap: theme.spacing(1.25, 2.75),
    padding: theme.spacing(1.25, 1.5),
  },
  group: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(0.5),
  },
  groupLabel: {
    fontSize: 10,
    letterSpacing: '0.1em',
    textTransform: 'uppercase',
    color: theme.palette.text.secondary,
  },
  pad: {
    display: 'grid',
    gap: 3,
  },
  padButton: {
    minWidth: 0,
    width: PAD,
    height: PAD,
    padding: 0,
    fontSize: 16,
    borderColor: theme.palette.divider,
    color: theme.palette.text.primary,
  },
  padValue: {
    width: PAD,
    height: PAD,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    fontSize: 11,
    color: theme.palette.text.secondary,
    fontVariantNumeric: 'tabular-nums',
  },
  quick: {
    flex: 1,
    minWidth: 220,
    display: 'grid',
    gridTemplateColumns: 'repeat(auto-fill, minmax(104px, 1fr))',
    gap: theme.spacing(0.75),
    alignContent: 'start',
  },
  footer: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1),
    padding: theme.spacing(1, 1.5),
    borderTop: `1px solid ${theme.palette.divider}`,
  },
  chips: {
    flex: 1,
    minWidth: 0,
    display: 'flex',
    flexWrap: 'wrap',
    gap: theme.spacing(0.5),
  },
  locked: {
    position: 'absolute',
    inset: '42px 0 0 0',
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
    gap: theme.spacing(1.25),
    background: alpha(theme.palette.background.paper, 0.7),
    zIndex: 1,
  },
}));

const PadButton = ({ label, title, onClick, disabled }) => {
  const { classes } = useStyles();
  return (
    <Button
      variant="outlined"
      className={classes.padButton}
      title={title}
      aria-label={title}
      onClick={onClick}
      disabled={disabled}
    >
      {label}
    </Button>
  );
};

const PadValue = ({ children }) => {
  const { classes } = useStyles();
  return <span className={classes.padValue}>{children}</span>;
};

/**
 * @param quickCommands [{ type, label, danger }] already filtered to what the device supports
 */
const ManualControlPanel = ({
  deviceName,
  locked,
  onToggleLock,
  armed,
  canMove,
  hasGimbal,
  gimbal,
  step,
  onStepChange,
  pending,
  onNudge,
  onCenterGimbal,
  onClear,
  onApply,
  quickCommands,
  onQuickCommand,
}) => {
  const { classes, theme } = useStyles();
  const size = STEPS[step];
  const chips = pendingChips(pending);
  const anyPending = hasPending(pending);
  const n = (delta) => () => onNudge(delta);

  return (
    <Paper variant="outlined" square className={classes.root}>
      <div className={classes.header}>
        <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
          Manual control
        </Typography>
        <Typography variant="caption" color="textSecondary" className={classes.hint}>
          {locked
            ? 'Supervisor view'
            : 'Keys: WASD move · R/F alt · Q/E yaw · IJKL gimbal · Enter apply · Esc clear'}
        </Typography>
        {armed != null && (
          <Typography
            variant="caption"
            sx={{
              fontWeight: 600,
              color: armed ? theme.palette.success.main : theme.palette.text.secondary,
            }}
          >
            {armed ? 'Armed' : 'Disarmed'}
          </Typography>
        )}
        <Button size="small" variant={locked ? 'contained' : 'outlined'} onClick={onToggleLock}>
          {locked ? 'Take control' : 'Release'}
        </Button>
      </div>

      <div className={classes.body}>
        {canMove && (
          <>
            <div className={classes.group}>
              <span className={classes.groupLabel}>Position</span>
              <div className={classes.pad} style={{ gridTemplateColumns: `repeat(3, ${PAD}px)` }}>
                <PadButton label="↖" title="Forward-left" onClick={n({ fwd: 1, right: -1 })} />
                <PadButton label="↑" title="Forward (W)" onClick={n({ fwd: 1 })} />
                <PadButton label="↗" title="Forward-right" onClick={n({ fwd: 1, right: 1 })} />
                <PadButton label="←" title="Left (A)" onClick={n({ right: -1 })} />
                <PadValue>{`${size.m} m`}</PadValue>
                <PadButton label="→" title="Right (D)" onClick={n({ right: 1 })} />
                <PadButton label="↙" title="Back-left" onClick={n({ fwd: -1, right: -1 })} />
                <PadButton label="↓" title="Back (S)" onClick={n({ fwd: -1 })} />
                <PadButton label="↘" title="Back-right" onClick={n({ fwd: -1, right: 1 })} />
              </div>
            </div>
            <div className={classes.group}>
              <span className={classes.groupLabel}>Alt · Yaw</span>
              <div className={classes.pad} style={{ gridTemplateColumns: `repeat(2, ${PAD}px)` }}>
                <PadButton label="▲" title="Up (R)" onClick={n({ up: 1 })} />
                <PadButton label="↺" title="Yaw left (Q)" onClick={n({ yaw: -1 })} />
                <PadValue>{`${formatSigned(pending.up)}m`}</PadValue>
                <PadValue>{`${formatSigned(pending.yaw)}°`}</PadValue>
                <PadButton label="▼" title="Down (F)" onClick={n({ up: -1 })} />
                <PadButton label="↻" title="Yaw right (E)" onClick={n({ yaw: 1 })} />
              </div>
            </div>
          </>
        )}
        {hasGimbal && (
          <div className={classes.group}>
            <span className={classes.groupLabel}>
              {`Gimbal · P ${gimbal.pitch + pending.gp}° · Y ${formatSigned(gimbal.yaw + pending.gy) || 0}°`}
            </span>
            <div className={classes.pad} style={{ gridTemplateColumns: `repeat(3, ${PAD}px)` }}>
              <PadValue />
              <PadButton label="▲" title="Pitch up (I)" onClick={n({ gp: 1 })} />
              <PadValue />
              <PadButton label="◀" title="Yaw left (J)" onClick={n({ gy: -1 })} />
              <PadButton label="⟲" title="Center gimbal" onClick={onCenterGimbal} />
              <PadButton label="▶" title="Yaw right (L)" onClick={n({ gy: 1 })} />
              <PadValue />
              <PadButton label="▼" title="Pitch down (K)" onClick={n({ gp: -1 })} />
              <PadValue />
            </div>
          </div>
        )}
        {quickCommands.length > 0 && (
          <div className={classes.group} style={{ flex: 1 }}>
            <span className={classes.groupLabel}>Quick commands</span>
            <div className={classes.quick}>
              {quickCommands.map(({ type, label, danger }) => (
                <Button
                  key={type}
                  size="small"
                  variant={danger ? 'contained' : 'outlined'}
                  color={danger ? 'error' : 'inherit'}
                  onClick={() => onQuickCommand(type)}
                >
                  {label}
                </Button>
              ))}
            </div>
          </div>
        )}
      </div>

      {(canMove || hasGimbal) && (
        <div className={classes.footer}>
          <ToggleButtonGroup
            size="small"
            exclusive
            value={step}
            onChange={(_, value) => value != null && onStepChange(value)}
            sx={{ height: 30 }}
          >
            {STEPS.map((s, index) => (
              <ToggleButton key={s.label} value={index} sx={{ whiteSpace: 'nowrap' }}>
                {s.label}
              </ToggleButton>
            ))}
          </ToggleButtonGroup>
          <div className={classes.chips}>
            {chips.length ? (
              chips.map((chip) => (
                <Chip key={chip} size="small" color="primary" variant="outlined" label={chip} />
              ))
            ) : (
              <Typography variant="caption" color="textSecondary">
                No pending movement — use the buttons or keys
              </Typography>
            )}
          </div>
          <Button variant="outlined" color="inherit" disabled={!anyPending} onClick={onClear}>
            Clear
          </Button>
          <Button variant="contained" disabled={!anyPending} onClick={onApply}>
            Apply ↵
          </Button>
        </div>
      )}

      {locked && (
        <div className={classes.locked}>
          <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
            Monitoring only
          </Typography>
          <Typography variant="body2" color="textSecondary">
            {`Take control to move ${deviceName ?? 'the UAV'}. A running mission will pause.`}
          </Typography>
          <Button variant="contained" onClick={onToggleLock}>
            Take control
          </Button>
        </div>
      )}
    </Paper>
  );
};

export default ManualControlPanel;
