import { useEffect, useState } from 'react';
import { Alert, Box, Button, List, ListItemButton, Stack, Typography } from '@mui/material';
import FactCheckIcon from '@mui/icons-material/FactCheck';
import { checkMission } from '../../common/util/fetchs';
import { useCatch } from '../../reactHelper';

// Where a problem is, as the server locates it: `T1 · wp 3 · action.gimbal` (wp is 0-based on the
// wire, 1-based on screen, like server missionValidator.formatProblem).
const whereOf = ({ task, wp, field }) =>
  [task, wp != null ? `wp ${wp + 1}` : null, field].filter(Boolean).join(' · ');

// The Validate button of the mission editor and what it found. The rules are the server's
// (models/mission/missionValidator.js), the same a load is refused with: nothing is decided here.
// A result belongs to the mission it checked, so any edit drops it.
const MissionValidation = ({ mission, onSelect }) => {
  const [result, setResult] = useState(null);
  const [checking, setChecking] = useState(false);

  useEffect(() => {
    setResult(null);
  }, [mission.route]);

  const handleCheck = useCatch(async () => {
    setChecking(true);
    try {
      setResult(await checkMission(mission));
    } finally {
      setChecking(false);
    }
  });

  // The route a problem points at, by its task id, so a click opens it (and its waypoint).
  const select = (problem) => {
    const routeIndex = mission.route.findIndex((r) => r.task_id === problem.task);
    if (routeIndex >= 0) onSelect(routeIndex, problem.wp);
  };

  const errors = result?.problems.filter((p) => p.level === 'error') ?? [];
  const warnings = result?.problems.filter((p) => p.level === 'warning') ?? [];

  return (
    <Stack spacing={1}>
      <Box sx={{ textAlign: 'center' }}>
        <Button
          variant="outlined"
          size="large"
          sx={{ width: '80%' }}
          startIcon={<FactCheckIcon />}
          disabled={checking}
          onClick={handleCheck}
        >
          {checking ? 'Validating…' : 'Validate mission'}
        </Button>
      </Box>
      {result && errors.length === 0 && (
        <Alert severity={warnings.length ? 'warning' : 'success'}>
          {warnings.length
            ? 'It can be loaded, with warnings'
            : 'Valid: every device can fly what it is given'}
        </Alert>
      )}
      {result && errors.length > 0 && (
        <Alert severity="error">
          {`${errors.length} problem${errors.length > 1 ? 's' : ''}: the server will refuse to load it`}
        </Alert>
      )}
      {result?.problems.length > 0 && (
        <List dense disablePadding>
          {[...errors, ...warnings].map((problem, i) => (
            <ListItemButton
              key={`${whereOf(problem)}-${i}`}
              onClick={() => select(problem)}
              sx={{ alignItems: 'flex-start', px: 1 }}
            >
              <Box>
                <Typography
                  variant="caption"
                  color={problem.level === 'error' ? 'error' : 'warning.main'}
                  sx={{ fontWeight: 600 }}
                >
                  {whereOf(problem) || 'mission'}
                </Typography>
                <Typography variant="body2">{problem.message}</Typography>
              </Box>
            </ListItemButton>
          ))}
        </List>
      )}
    </Stack>
  );
};

export default MissionValidation;
