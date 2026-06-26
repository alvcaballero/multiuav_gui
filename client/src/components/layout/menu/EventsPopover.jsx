import { useNavigate } from 'react-router-dom';
import { useSelector } from 'react-redux';
import { grey, red, orange, blue } from '@mui/material/colors';
import { Box, Button, Divider, Popover, Typography } from '@mui/material';
import CircleIcon from '@mui/icons-material/Circle';
import NotificationsIcon from '@mui/icons-material/Notifications';
import { makeStyles } from 'tss-react/mui';

const EVENT_COLORS = {
  error: red[600],
  warning: orange[700],
  info: blue[600],
  default: grey[500],
};

function relativeTime(ts) {
  const diff = Math.floor((Date.now() - ts) / 1000);
  if (diff < 60) return `${diff}s ago`;
  if (diff < 3600) return `${Math.floor(diff / 60)}m ago`;
  return `${Math.floor(diff / 3600)}h ago`;
}

const useStyles = makeStyles()(() => ({
  eventRow: {
    display: 'flex',
    alignItems: 'flex-start',
    gap: '8px',
    padding: '5px 12px',
    borderBottom: `1px solid ${grey[100]}`,
    '&:last-child': { borderBottom: 'none' },
  },
  eventMsg: {
    fontSize: '12px',
    lineHeight: 1.4,
    color: grey[800],
    flex: 1,
  },
  eventTime: {
    fontSize: '11px',
    color: grey[500],
    whiteSpace: 'nowrap',
    marginTop: '1px',
  },
}));

const EventsPopover = ({ anchor, onClose }) => {
  const { classes } = useStyles();
  const navigate = useNavigate();

  const events = useSelector((state) => state.events.items);
  const recentEvents = events.slice(0, 8);

  return (
    <Popover
      open={Boolean(anchor)}
      anchorEl={anchor}
      onClose={onClose}
      anchorOrigin={{ vertical: 'bottom', horizontal: 'right' }}
      transformOrigin={{ vertical: 'top', horizontal: 'right' }}
      PaperProps={{ sx: { width: 360, mt: 0.5 } }}
    >
      <Box>
        <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', px: 1.5, py: 0.75 }}>
          <Typography variant="caption" sx={{ fontWeight: 600, letterSpacing: 0.5, textTransform: 'uppercase', color: 'text.secondary' }}>
            Recent Events
          </Typography>
          <Typography variant="caption" sx={{ color: grey[400], fontSize: 10 }}>
            {`showing last ${recentEvents.length} of ${events.length}`}
          </Typography>
        </Box>

        <Divider />

        {recentEvents.length === 0 ? (
          <Typography sx={{ px: 1.5, py: 2, fontSize: 13, color: 'text.secondary' }}>
            No events recorded.
          </Typography>
        ) : (
          recentEvents.map((ev) => (
            <div key={ev.id} className={classes.eventRow}>
              <CircleIcon
                sx={{
                  fontSize: 7,
                  mt: '4px',
                  flexShrink: 0,
                  color: EVENT_COLORS[ev.type] ?? EVENT_COLORS.default,
                }}
              />
              <Typography className={classes.eventMsg}>
                {ev.attributes?.message ?? ev.type}
              </Typography>
              <Typography className={classes.eventTime}>
                {relativeTime(ev.eventTime)}
              </Typography>
            </div>
          ))
        )}

        <Divider />
        <Box sx={{ px: 1, py: 0.75, display: 'flex', justifyContent: 'flex-end' }}>
          <Button
            size="small"
            endIcon={<NotificationsIcon sx={{ fontSize: 13 }} />}
            onClick={() => { onClose(); navigate('/reports/events'); }}
            sx={{ fontSize: 11, textTransform: 'none', color: grey[600] }}
          >
            View all events
          </Button>
        </Box>
      </Box>
    </Popover>
  );
};

export default EventsPopover;
