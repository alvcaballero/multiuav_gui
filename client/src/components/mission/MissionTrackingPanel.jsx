import { useSelector, useDispatch } from 'react-redux';
import {
  Box,
  Chip,
  Collapse,
  LinearProgress,
  List,
  ListItemButton,
  ListItemText,
  Tooltip,
  Typography,
} from '@mui/material';
import WarningAmberIcon from '@mui/icons-material/WarningAmber';
import { activeMissionsActions, missionActions } from '../../store';
import { map } from '../../map/core/MapView';

const STATUS_COLOR = {
  init: 'default',
  commanded: 'info',
  running: 'primary',
  completed: 'success',
};

const STATUS_LABEL = {
  init: 'Init',
  commanded: 'Sent',
  running: 'Running',
  completed: 'Done',
};

const ANOMALY_LABEL = {
  DEVIATION:      'Deviating from route',
  RTH_SUSPECTED:  'Return to home suspected',
  ON_GROUND:      'UAV on ground',
  DISARMED:       'UAV disarmed',
  TELEMETRY_GAP:  'Telemetry gap — WP estimate may be ahead',
};

const RouteRow = ({ route, devicesMap }) => {
  const device = Object.values(devicesMap).find((d) => d.id === route.deviceId);
  const name = device?.name ?? `UAV ${route.deviceId}`;
  const pct = route.totalWp > 0 ? Math.round((route.currentWp / route.totalWp) * 100) : 0;
  const hasAnomaly = route.anomalies?.length > 0;
  const anomalyText = route.anomalies?.map((a) => ANOMALY_LABEL[a] ?? a).join(' · ') ?? '';
  const showEstimate = route.wpEstimate !== null && route.wpEstimate !== route.currentWp;

  return (
    <Box sx={{ px: 2, py: 0.5 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 0.25 }}>
        <Box sx={{ display: 'flex', alignItems: 'center', flex: 1, minWidth: 0 }}>
          {hasAnomaly && (
            <Tooltip title={anomalyText} placement="top" arrow>
              <WarningAmberIcon sx={{ fontSize: 14, color: 'warning.main', mr: 0.5, flexShrink: 0 }} />
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
              <Typography variant="caption" color="warning.main" sx={{ ml: 0.5, whiteSpace: 'nowrap' }}>
                (~{route.wpEstimate})
              </Typography>
            </Tooltip>
          )}
          <Chip
            label={STATUS_LABEL[route.status] ?? route.status}
            color={hasAnomaly ? 'warning' : (STATUS_COLOR[route.status] ?? 'default')}
            size="small"
            sx={{ ml: 1, height: 18, fontSize: '0.6rem' }}
          />
        </Box>
      </Box>
      <LinearProgress
        variant="determinate"
        value={pct}
        color={route.status === 'completed' ? 'success' : hasAnomaly ? 'warning' : 'primary'}
        sx={{ height: 4, borderRadius: 2 }}
      />
    </Box>
  );
};

const MissionTrackingPanel = () => {
  const dispatch = useDispatch();
  const { items, selectedMissionId } = useSelector((state) => state.activeMissions);
  const devicesMap = useSelector((state) => state.devices.items);
  const currentMissionName = useSelector((state) => state.mission.name);

  const missions = Object.values(items);

  if (missions.length === 0) {
    return (
      <Box sx={{ px: 2, py: 1 }}>
        <Typography variant="caption" color="text.secondary">
          No active missions
        </Typography>
      </Box>
    );
  }

  const flyToMission = (missionData) => {
    const firstWp = missionData?.route?.[0]?.wp?.[0];
    if (!firstWp || !map) return;
    const [lat, lon] = Array.isArray(firstWp.pos) ? firstWp.pos : [firstWp.pos.lat, firstWp.pos.lon];
    map.easeTo({ center: [lon, lat], zoom: Math.max(map.getZoom(), 14) });
  };

  const handleSelect = async (mission) => {
    const id = mission.id;
    // Toggle collapse
    dispatch(activeMissionsActions.selectMission(selectedMissionId === id ? null : id));

    // Already selected — just collapse, don't reload
    if (selectedMissionId === id) return;

    // If this mission is already loaded in the mission store, just fly to it
    if (currentMissionName === mission.name) {
      flyToMission({ route: Object.values(mission.routes) });
      return;
    }

    // Need planId to fetch missionData — get it from the DB record
    try {
      const res = await fetch(`/api/missions?id=${id}`);
      if (!res.ok) return;
      const record = await res.json();
      const planId = Array.isArray(record) ? record[0]?.planId : record?.planId;
      if (!planId) return;

      const planRes = await fetch(`/api/missions/plans/${planId}`);
      if (!planRes.ok) return;
      const plan = await planRes.json();
      const missionData = plan.missionData;
      if (!missionData) return;

      dispatch(missionActions.updateMission({ ...missionData, name: mission.name }));
      flyToMission(missionData);
    } catch (_) {
      // fetch failed — silently skip map load
    }
  };

  return (
    <List dense disablePadding>
      {missions.map((mission) => {
        const isSelected = selectedMissionId === mission.id;
        const routes = Object.values(mission.routes);

        return (
          <Box key={mission.id}>
            <ListItemButton
              selected={isSelected}
              onClick={() => handleSelect(mission)}
              sx={{ py: 0.75 }}
            >
              <ListItemText
                primary={
                  <Typography variant="body2" noWrap fontWeight={isSelected ? 600 : 400}>
                    {mission.name}
                  </Typography>
                }
                secondary={
                  <Typography variant="caption" color="text.secondary">
                    {new Date(mission.initTime).toLocaleTimeString()}
                  </Typography>
                }
              />
              <Chip
                label={STATUS_LABEL[mission.status] ?? mission.status}
                color={STATUS_COLOR[mission.status] ?? 'default'}
                size="small"
                sx={{ ml: 1 }}
              />
            </ListItemButton>

            <Collapse in={isSelected} unmountOnExit>
              <Box sx={{ bgcolor: 'action.hover', pb: 0.5 }}>
                {routes.length === 0 ? (
                  <Typography variant="caption" sx={{ px: 2, py: 0.5, display: 'block' }} color="text.secondary">
                    No routes yet
                  </Typography>
                ) : (
                  routes.map((r) => <RouteRow key={r.deviceId} route={r} devicesMap={devicesMap} />)
                )}
              </Box>
            </Collapse>
          </Box>
        );
      })}
    </List>
  );
};

export default MissionTrackingPanel;
