import React, { useCallback } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { Box, Chip, Collapse, List, ListItemButton, ListItemText, Typography } from '@mui/material';
import { activeMissionsActions } from '../../store';
import { missionStyle } from '../../shared/missionStatus';
import { loadMissionPlanToEditor } from '../../services/missionPlanLoader';
import RouteTrackingRow from './RouteTrackingRow';

// One mission row, extracted (like RouteTrackingRow) so it only re-renders
// when its own mission/selection data changes, not on every telemetry tick
// that touches a sibling mission in the list.
const MissionTrackingRow = React.memo(({ mission, isSelected, devicesMap, onSelect }) => {
  const routes = Object.values(mission.routes);

  return (
    <Box>
      <ListItemButton selected={isSelected} onClick={() => onSelect(mission)} sx={{ py: 0.75 }}>
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
          label={missionStyle(mission.status).label}
          size="small"
          sx={{ ml: 1, backgroundColor: missionStyle(mission.status).color, color: '#fff' }}
        />
      </ListItemButton>

      <Collapse in={isSelected} unmountOnExit>
        <Box sx={{ bgcolor: 'action.hover', pb: 0.5 }}>
          {routes.length === 0 ? (
            <Typography
              variant="caption"
              sx={{ px: 2, py: 0.5, display: 'block' }}
              color="text.secondary"
            >
              No routes yet
            </Typography>
          ) : (
            routes.map((r) => (
              <RouteTrackingRow key={r.deviceId} route={r} devicesMap={devicesMap} />
            ))
          )}
        </Box>
      </Collapse>
    </Box>
  );
});

const MissionTrackingPanel = () => {
  const dispatch = useDispatch();
  const { items, selectedMissionId } = useSelector((state) => state.activeMissions);
  const devicesMap = useSelector((state) => state.devices.items);
  const currentMissionName = useSelector((state) => state.mission.name);

  const missions = Object.values(items);

  const handleSelect = useCallback(
    async (mission) => {
      const id = mission.id;
      // Toggle collapse
      dispatch(activeMissionsActions.selectMission(selectedMissionId === id ? null : id));

      // Already selected — just collapse, don't reload
      if (selectedMissionId === id) return;

      // If this mission is already loaded in the mission store, don't re-fetch —
      // just fly to it (the editor's route already has the data we need).
      if (currentMissionName === mission.name) return;

      await loadMissionPlanToEditor(id, dispatch);
    },
    [dispatch, selectedMissionId, currentMissionName],
  );

  if (missions.length === 0) {
    return (
      <Box sx={{ px: 2, py: 1 }}>
        <Typography variant="caption" color="text.secondary">
          No active missions
        </Typography>
      </Box>
    );
  }

  return (
    <List dense disablePadding>
      {missions.map((mission) => (
        <MissionTrackingRow
          key={mission.id}
          mission={mission}
          isSelected={selectedMissionId === mission.id}
          devicesMap={devicesMap}
          onSelect={handleSelect}
        />
      ))}
    </List>
  );
};

export default MissionTrackingPanel;
