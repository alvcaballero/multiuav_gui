import { useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import {
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Button,
  ToggleButton,
  ToggleButtonGroup,
  TextField,
  Typography,
  Box,
  MenuItem,
  Select,
  FormControl,
  InputLabel,
} from '@mui/material';
import RotateRightIcon from '@mui/icons-material/RotateRight';
import OpenWithIcon from '@mui/icons-material/OpenWith';
import { missionActions } from '../../store';

const MissionTransformDialog = ({ open, onClose }) => {
  const dispatch = useDispatch();
  const routes = useSelector((state) => state.mission.route);

  const [mode, setMode] = useState('rotate'); // 'rotate' | 'translate'
  const [routeIndex, setRouteIndex] = useState(-1);

  // Rotate state
  const [angleDeg, setAngleDeg] = useState('');

  // Translate state
  const [deltaLat, setDeltaLat] = useState('');
  const [deltaLng, setDeltaLng] = useState('');

  const handleApply = () => {
    if (mode === 'rotate') {
      const angle = parseFloat(angleDeg);
      if (isNaN(angle)) return;
      dispatch(missionActions.rotateMission({ angleDeg: angle, routeIndex }));
    } else {
      const dLat = parseFloat(deltaLat);
      const dLng = parseFloat(deltaLng);
      if (isNaN(dLat) || isNaN(dLng)) return;
      dispatch(missionActions.translateMission({ deltaLat: dLat, deltaLng: dLng, routeIndex }));
    }
    onClose();
  };

  const handleClose = () => {
    setAngleDeg('');
    setDeltaLat('');
    setDeltaLng('');
    onClose();
  };

  const isApplyDisabled = () => {
    if (mode === 'rotate') return angleDeg === '' || isNaN(parseFloat(angleDeg));
    return (
      deltaLat === '' ||
      deltaLng === '' ||
      isNaN(parseFloat(deltaLat)) ||
      isNaN(parseFloat(deltaLng))
    );
  };

  return (
    <Dialog open={open} onClose={handleClose} maxWidth="xs" fullWidth>
      <DialogTitle>Transform Mission</DialogTitle>
      <DialogContent>
        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2, mt: 1 }}>
          {/* Mode selector */}
          <ToggleButtonGroup
            value={mode}
            exclusive
            onChange={(_, val) => val && setMode(val)}
            fullWidth
            size="small"
          >
            <ToggleButton value="rotate">
              <RotateRightIcon sx={{ mr: 1 }} />
              Rotate
            </ToggleButton>
            <ToggleButton value="translate">
              <OpenWithIcon sx={{ mr: 1 }} />
              Translate
            </ToggleButton>
          </ToggleButtonGroup>

          {/* Route selector */}
          <FormControl size="small" fullWidth>
            <InputLabel>Route</InputLabel>
            <Select
              value={routeIndex}
              label="Route"
              onChange={(e) => setRouteIndex(e.target.value)}
            >
              <MenuItem value={-1}>All routes</MenuItem>
              {routes.map((route, i) => (
                <MenuItem key={route.id} value={i}>
                  {route.name || `Route ${i + 1}`}
                </MenuItem>
              ))}
            </Select>
          </FormControl>

          {/* Rotate inputs */}
          {mode === 'rotate' && (
            <>
              <Typography variant="caption" color="text.secondary">
                Rotation around the centroid of the selected routes. Positive = counter-clockwise.
              </Typography>
              <TextField
                label="Angle (degrees)"
                type="number"
                size="small"
                value={angleDeg}
                onChange={(e) => setAngleDeg(e.target.value)}
                slotProps={{ htmlInput: { step: 1 } }}
                fullWidth
              />
            </>
          )}

          {/* Translate inputs */}
          {mode === 'translate' && (
            <>
              <Typography variant="caption" color="text.secondary">
                Offset in decimal degrees. Use negative values to go South/West.
              </Typography>
              <TextField
                label="ΔLat (degrees)"
                type="number"
                size="small"
                value={deltaLat}
                onChange={(e) => setDeltaLat(e.target.value)}
                slotProps={{ htmlInput: { step: 0.0001 } }}
                fullWidth
              />
              <TextField
                label="ΔLng (degrees)"
                type="number"
                size="small"
                value={deltaLng}
                onChange={(e) => setDeltaLng(e.target.value)}
                slotProps={{ htmlInput: { step: 0.0001 } }}
                fullWidth
              />
            </>
          )}
        </Box>
      </DialogContent>
      <DialogActions>
        <Button onClick={handleClose}>Cancel</Button>
        <Button onClick={handleApply} variant="contained" disabled={isApplyDisabled()}>
          Apply
        </Button>
      </DialogActions>
    </Dialog>
  );
};

export default MissionTransformDialog;
