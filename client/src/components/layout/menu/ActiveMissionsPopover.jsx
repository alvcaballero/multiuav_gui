import { Box, Divider, Popover, Typography } from '@mui/material';
import MissionTrackingPanel from '../../mission/MissionTrackingPanel';

const ActiveMissionsPopover = ({ anchor, onClose }) => (
  <Popover
    open={Boolean(anchor)}
    anchorEl={anchor}
    onClose={onClose}
    anchorOrigin={{ vertical: 'bottom', horizontal: 'right' }}
    transformOrigin={{ vertical: 'top', horizontal: 'right' }}
    PaperProps={{ sx: { width: 340, mt: 0.5 } }}
  >
    <Box>
      <Typography
        variant="caption"
        sx={{
          px: 1.5,
          py: 0.75,
          display: 'block',
          fontWeight: 600,
          letterSpacing: 0.5,
          textTransform: 'uppercase',
          color: 'text.secondary',
        }}
      >
        Active Missions
      </Typography>
      <Divider />
      <MissionTrackingPanel />
    </Box>
  </Popover>
);

export default ActiveMissionsPopover;
