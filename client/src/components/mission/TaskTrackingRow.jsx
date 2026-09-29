import { Box, Chip, LinearProgress, Tooltip, Typography } from '@mui/material';
import WarningAmberIcon from '@mui/icons-material/WarningAmber';
import { amber } from '@mui/material/colors';
import { taskStyle } from '../../shared/missionStatus';

// Anomaly overrides the task's own status color — an anomalous task is worth
// flagging regardless of where it is in its lifecycle.
const ANOMALY_STYLE = { color: amber[700], label: 'Anomaly' };

const ANOMALY_LABEL = {
  DEVIATION: 'Deviating from route',
  RTH_SUSPECTED: 'Return to home suspected',
  ON_GROUND: 'UAV on ground',
  DISARMED: 'UAV disarmed',
  TELEMETRY_GAP: 'Telemetry gap — WP estimate may be ahead',
};

// Tasks converted from a legacy route carry this generic action: not worth showing.
const LEGACY_ACTION = 'ROUTE';

// Live tracking row for one task: key, device, WP progress, anomalies and status chip.
// Shared by MissionTrackingPanel (active missions list) and MissionDetailPopover.
// `task` comes from state.activeMissions[missionId].tasks (WebSocket-fed).
const TaskTrackingRow = ({ task, devicesMap }) => {
  const device = Object.values(devicesMap).find((d) => d.id === task.deviceId);
  const deviceName = device?.name ?? (task.deviceId != null ? `UAV ${task.deviceId}` : 'no device');
  const title = task.taskKey ? `${task.taskKey} · ${deviceName}` : deviceName;
  const pct = task.totalWp > 0 ? Math.round((task.currentWp / task.totalWp) * 100) : 0;
  const hasAnomaly = task.anomalies?.length > 0;
  const anomalyText = task.anomalies?.map((a) => ANOMALY_LABEL[a] ?? a).join(' · ') ?? '';
  const showEstimate =
    task.wpEstimate !== null && task.wpEstimate !== undefined && task.wpEstimate !== task.currentWp;
  const waitingFor = task.status === 'init' ? (task.dependsOn ?? []) : [];
  const style = hasAnomaly ? ANOMALY_STYLE : taskStyle(task.status);

  return (
    <Box sx={{ px: 2, py: 0.5 }}>
      <Box
        sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 0.25 }}
      >
        <Box sx={{ display: 'flex', alignItems: 'center', flex: 1, minWidth: 0 }}>
          {hasAnomaly && (
            <Tooltip title={anomalyText} placement="top" arrow>
              <WarningAmberIcon
                sx={{ fontSize: 14, color: 'warning.main', mr: 0.5, flexShrink: 0 }}
              />
            </Tooltip>
          )}
          <Typography variant="caption" noWrap>
            {title}
          </Typography>
          {task.action && task.action !== LEGACY_ACTION && (
            <Typography variant="caption" color="text.secondary" noWrap sx={{ ml: 0.5 }}>
              {task.action}
            </Typography>
          )}
        </Box>
        <Box sx={{ display: 'flex', alignItems: 'center', flexShrink: 0, ml: 1 }}>
          <Typography variant="caption" color="text.secondary" sx={{ whiteSpace: 'nowrap' }}>
            {task.currentWp}/{task.totalWp} WP
          </Typography>
          {showEstimate && (
            <Tooltip title={`Time estimate: WP ${task.wpEstimate}`} placement="top" arrow>
              <Typography
                variant="caption"
                color="warning.main"
                sx={{ ml: 0.5, whiteSpace: 'nowrap' }}
              >
                (~{task.wpEstimate})
              </Typography>
            </Tooltip>
          )}
          <Chip
            label={style.label}
            size="small"
            sx={{
              ml: 1,
              height: 18,
              fontSize: '0.6rem',
              backgroundColor: style.color,
              color: '#fff',
            }}
          />
        </Box>
      </Box>
      {waitingFor.length > 0 && (
        <Typography variant="caption" color="text.secondary" sx={{ display: 'block' }}>
          waits for {waitingFor.join(', ')}
        </Typography>
      )}
      <LinearProgress
        variant="determinate"
        value={pct}
        color={task.status === 'complete' ? 'success' : hasAnomaly ? 'warning' : 'primary'}
        sx={{ height: 4, borderRadius: 2 }}
      />
    </Box>
  );
};

export default TaskTrackingRow;
