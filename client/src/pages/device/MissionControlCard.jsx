import { Button, LinearProgress, Paper, Typography } from '@mui/material';
import { alpha } from '@mui/material/styles';
import { makeStyles } from 'tss-react/mui';
import dayjs from 'dayjs';

const useStyles = makeStyles()((theme) => ({
  root: {
    padding: theme.spacing(1.5, 1.75),
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(1.25),
    fontVariantNumeric: 'tabular-nums',
  },
  row: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    gap: theme.spacing(1),
  },
  tag: {
    fontSize: 11,
    fontWeight: 600,
    padding: '2px 8px',
  },
  progress: {
    height: 6,
    '& .MuiLinearProgress-bar': { transition: 'transform .8s' },
  },
  buttons: {
    display: 'grid',
    gridTemplateColumns: 'repeat(3, minmax(0, 1fr))',
    gap: theme.spacing(1),
  },
}));

// Task status (server TASK_STATUS) → label + palette key. "Paused" is not a task status: it comes from
// the vehicle (or from this page having sent the pause), see useMissionActions.
const STATUS = {
  paused: ['Paused', 'warning'],
  running: ['Running', 'success'],
  commanded: ['Starting', 'success'],
  loaded: ['Loaded', 'info'],
  init: ['Waiting', 'info'],
};

const MissionControlCard = ({ mission, task, paused, actions }) => {
  const { classes, theme } = useStyles();
  const [label, paletteKey] = STATUS[paused ? 'paused' : task.status] ?? [task.status, 'info'];
  const color = theme.palette[paletteKey].main;
  const total = task.totalWp ?? 0;
  const current = Math.min(task.currentWp ?? 0, total);
  const percent = total ? Math.round((current / total) * 100) : 0;
  const active = task.status === 'running' || paused;

  return (
    <Paper variant="outlined" square className={classes.root}>
      <div className={classes.row}>
        <Typography variant="overline" color="primary" sx={{ lineHeight: 1.2 }}>
          Mission
        </Typography>
        <span className={classes.tag} style={{ color, background: alpha(color, 0.13) }}>
          {label}
        </span>
      </div>
      <div>
        <Typography variant="h6" sx={{ lineHeight: 1.2 }}>
          {mission.name || `Mission ${mission.id}`}
        </Typography>
        <Typography variant="caption" color="textSecondary">
          {[
            `#${mission.id}`,
            `task ${task.taskKey ?? task.id}${task.action ? ` · ${task.action}` : ''}`,
            mission.initTime && `started ${dayjs(mission.initTime).format('HH:mm')}`,
          ]
            .filter(Boolean)
            .join(' · ')}
        </Typography>
      </div>
      <div>
        <div className={classes.row}>
          <Typography variant="body2">
            {total ? `WP ${current} / ${total}` : 'No waypoint data'}
          </Typography>
          <Typography variant="body2" color="textSecondary">{`${percent}%`}</Typography>
        </div>
        <LinearProgress variant="determinate" value={percent} className={classes.progress} />
      </div>
      <div className={classes.buttons}>
        <Button
          variant="outlined"
          disabled={!active || !(paused ? actions.resume : actions.pause)}
          onClick={paused ? actions.resume : actions.pause}
        >
          {paused ? 'Resume' : 'Pause'}
        </Button>
        <Button
          variant="outlined"
          color="inherit"
          disabled={!active || !actions.stop}
          onClick={actions.stop}
        >
          Stop
        </Button>
        <Button
          variant="contained"
          color="error"
          disabled={!active || !actions.abort}
          onClick={actions.abort}
        >
          Abort
        </Button>
      </div>
    </Paper>
  );
};

export default MissionControlCard;
