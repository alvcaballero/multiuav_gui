import { useState } from 'react';
import { useSelector } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import { grey, blue, green, orange } from '@mui/material/colors';
import { Box, Button, Chip, Collapse, Divider, Popover, Typography } from '@mui/material';
import RouteIcon from '@mui/icons-material/Route';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import PlaceIcon from '@mui/icons-material/Place';
import ModeEditIcon from '@mui/icons-material/ModeEdit';
import DeleteForeverIcon from '@mui/icons-material/DeleteForever';
import { makeStyles } from 'tss-react/mui';

const MISSION_STATUS_COLOR = {
  running: green[700],
  completed: grey[500],
  init: orange[700],
};

const useStyles = makeStyles()(() => ({
  panelHeader: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: '8px 12px 6px',
  },
  panelTitle: {
    fontSize: '11px',
    fontWeight: 700,
    letterSpacing: '0.5px',
    textTransform: 'uppercase',
    color: grey[500],
  },
  wpBadge: {
    display: 'inline-flex',
    alignItems: 'center',
    justifyContent: 'center',
    width: '18px',
    height: '18px',
    borderRadius: '50%',
    backgroundColor: grey[200],
    fontSize: '10px',
    fontWeight: 700,
    color: grey[700],
    flexShrink: 0,
  },
}));

const MissionDetailPopover = ({ anchor, onClose, onClear }) => {
  const { classes } = useStyles();
  const navigate = useNavigate();

  const mission = useSelector((state) => state.mission);
  const missionRoutes = useSelector((state) => state.mission.route);
  const activeMissions = useSelector((state) => state.activeMissions.items);

  const activeMissionsList = Object.values(activeMissions);
  const runningMission = activeMissionsList.find((m) => m.status === 'running');
  const currentStatus = runningMission?.status ?? activeMissionsList[0]?.status ?? null;

  const [expandedRoute, setExpandedRoute] = useState(null);

  function handleClose() {
    setExpandedRoute(null);
    onClose();
  }

  return (
    <Popover
      open={Boolean(anchor)}
      anchorEl={anchor}
      onClose={handleClose}
      anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
      transformOrigin={{ vertical: 'top', horizontal: 'center' }}
      PaperProps={{ sx: { width: 360, mt: 0.5 } }}
    >
      <Box>
        <div className={classes.panelHeader}>
          <Typography className={classes.panelTitle}>Mission details</Typography>
          {currentStatus && (
            <Chip
              label={currentStatus}
              size="small"
              sx={{
                height: '16px',
                fontSize: '10px',
                fontWeight: 700,
                backgroundColor: MISSION_STATUS_COLOR[currentStatus] ?? grey[400],
                color: '#fff',
              }}
            />
          )}
        </div>

        {mission.description && (
          <Typography sx={{ px: 1.5, pb: 0.5, fontSize: 12, color: grey[600] }}>
            {mission.description}
          </Typography>
        )}

        <Divider />

        {missionRoutes.length === 0 ? (
          <Typography sx={{ px: 1.5, py: 2, fontSize: 13, color: 'text.secondary' }}>
            No routes defined.
          </Typography>
        ) : (
          missionRoutes.map((route) => (
            <Box key={route.id}>
              <Box
                onClick={() => setExpandedRoute(expandedRoute === route.id ? null : route.id)}
                sx={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: 1,
                  px: 1.5,
                  py: 0.75,
                  cursor: 'pointer',
                  '&:hover': { backgroundColor: grey[50] },
                  borderBottom: `1px solid ${grey[100]}`,
                }}
              >
                <RouteIcon sx={{ fontSize: 15, color: grey[500], flexShrink: 0 }} />
                <Typography sx={{ fontSize: 12, fontWeight: 600, flex: 1, color: grey[800] }}>
                  {route.name || `Route ${route.id}`}
                </Typography>
                <Chip
                  label={route.uav || 'unassigned'}
                  size="small"
                  sx={{
                    height: '16px',
                    fontSize: '10px',
                    fontWeight: 600,
                    backgroundColor: route.uav ? blue[50] : grey[100],
                    color: route.uav ? blue[800] : grey[600],
                    border: `1px solid ${route.uav ? blue[200] : grey[300]}`,
                  }}
                />
                <Typography sx={{ fontSize: 11, color: grey[500], ml: 0.5, flexShrink: 0 }}>
                  {route.wp?.length ?? 0} wp
                </Typography>
                <ExpandMoreIcon
                  sx={{
                    fontSize: 16,
                    color: grey[400],
                    flexShrink: 0,
                    transform: expandedRoute === route.id ? 'rotate(180deg)' : 'none',
                    transition: 'transform 0.2s',
                  }}
                />
              </Box>

              <Collapse in={expandedRoute === route.id}>
                <Box sx={{ backgroundColor: grey[50], maxHeight: 160, overflowY: 'auto' }}>
                  {(route.wp ?? []).map((wp, idx) => (
                    <Box
                      key={idx}
                      sx={{
                        display: 'flex',
                        alignItems: 'center',
                        gap: 1,
                        px: 2,
                        py: 0.5,
                        borderBottom: `1px solid ${grey[100]}`,
                        '&:last-child': { borderBottom: 'none' },
                      }}
                    >
                      <span className={classes.wpBadge}>{idx + 1}</span>
                      <PlaceIcon sx={{ fontSize: 13, color: grey[400], flexShrink: 0 }} />
                      <Typography sx={{ fontSize: 11, color: grey[700], flex: 1 }}>
                        {wp.pos
                          ? `${Number(wp.pos[0]).toFixed(5)}, ${Number(wp.pos[1]).toFixed(5)}`
                          : '—'}
                      </Typography>
                      <Typography sx={{ fontSize: 11, color: grey[500], flexShrink: 0 }}>
                        {wp.pos?.[2] != null ? `${Number(wp.pos[2]).toFixed(1)} m` : ''}
                      </Typography>
                    </Box>
                  ))}
                </Box>
              </Collapse>
            </Box>
          ))
        )}

        <Divider />
        <Box sx={{ display: 'flex', gap: 1, p: 1, justifyContent: 'flex-end' }}>
          <Button
            size="small"
            startIcon={<ModeEditIcon sx={{ fontSize: 13 }} />}
            onClick={() => { handleClose(); navigate('/mission'); }}
            sx={{ fontSize: 11, textTransform: 'none' }}
          >
            Edit
          </Button>
          <Button
            size="small"
            color="error"
            startIcon={<DeleteForeverIcon sx={{ fontSize: 13 }} />}
            onClick={() => { handleClose(); onClear(); }}
            sx={{ fontSize: 11, textTransform: 'none' }}
          >
            Clear
          </Button>
        </Box>
      </Box>
    </Popover>
  );
};

export default MissionDetailPopover;
