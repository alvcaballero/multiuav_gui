import { useSelector, useDispatch } from 'react-redux';
import { Box, Chip, Collapse, List, ListItemButton, ListItemText, Typography } from '@mui/material';
import { activeMissionsActions } from '../../store';
import { missionStyle } from '../../shared/missionStatus';
import { loadMissionPlanToEditor } from '../../services/missionPlanLoader';
import RouteTrackingRow from './RouteTrackingRow';

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

  const handleSelect = async (mission) => {
    const id = mission.id;
    // Toggle collapse
    dispatch(activeMissionsActions.selectMission(selectedMissionId === id ? null : id));

    // Already selected — just collapse, don't reload
    if (selectedMissionId === id) return;

    // If this mission is already loaded in the mission store, don't re-fetch —
    // just fly to it (the editor's route already has the data we need).
    if (currentMissionName === mission.name) return;

    await loadMissionPlanToEditor(id, dispatch);
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
      })}
    </List>
  );
};

export default MissionTrackingPanel;
