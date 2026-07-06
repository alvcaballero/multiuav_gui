import { Box, Chip, LinearProgress, Tooltip, Typography } from '@mui/material';
import WarningAmberIcon from '@mui/icons-material/WarningAmber';
import { amber } from '@mui/material/colors';
import { routeStyle } from '../../shared/missionStatus';

// Anomaly overrides the route's own status color — an anomalous route is worth
// flagging regardless of where it is in its lifecycle.
const ANOMALY_STYLE = { color: amber[700], label: 'Anomaly' };

const ANOMALY_LABEL = {
  DEVIATION: 'Deviating from route',
  RTH_SUSPECTED: 'Return to home suspected',
  ON_GROUND: 'UAV on ground',
  DISARMED: 'UAV disarmed',
  TELEMETRY_GAP: 'Telemetry gap — WP estimate may be ahead',
};

// Live tracking row for one route: name, WP progress, anomalies and status chip.
// Shared by MissionTrackingPanel (active missions list) and MissionDetailPopover.
// `route` comes from state.activeMissions[missionId].routes (WebSocket-fed).
const RouteTrackingRow = ({ route, devicesMap }) => {
  const device = Object.values(devicesMap).find((d) => d.id === route.deviceId);
  const name = device?.name ?? `UAV ${route.deviceId}`;
  const pct = route.totalWp > 0 ? Math.round((route.currentWp / route.totalWp) * 100) : 0;
  const hasAnomaly = route.anomalies?.length > 0;
  const anomalyText = route.anomalies?.map((a) => ANOMALY_LABEL[a] ?? a).join(' · ') ?? '';
  const showEstimate =
    route.wpEstimate !== null &&
    route.wpEstimate !== undefined &&
    route.wpEstimate !== route.currentWp;
  const style = hasAnomaly ? ANOMALY_STYLE : routeStyle(route.status);

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
            {name}
          </Typography>
        </Box>
        <Box sx={{ display: 'flex', alignItems: 'center', flexShrink: 0, ml: 1 }}>
          <Typography variant="caption" color="text.secondary" sx={{ whiteSpace: 'nowrap' }}>
            {route.currentWp}/{route.totalWp} WP
          </Typography>
          {showEstimate && (
            <Tooltip title={`Time estimate: WP ${route.wpEstimate}`} placement="top" arrow>
              <Typography
                variant="caption"
                color="warning.main"
                sx={{ ml: 0.5, whiteSpace: 'nowrap' }}
              >
                (~{route.wpEstimate})
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
      <LinearProgress
        variant="determinate"
        value={pct}
        color={route.status === 'complete' ? 'success' : hasAnomaly ? 'warning' : 'primary'}
        sx={{ height: 4, borderRadius: 2 }}
      />
    </Box>
  );
};

export default RouteTrackingRow;
