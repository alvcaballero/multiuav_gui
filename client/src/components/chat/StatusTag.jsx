import { Chip } from '@mui/material';

// Named like the app's STATUS (common/theme/palette.js), with the same meaning; the shades are the
// handoff's.
const TONES = {
  // In progress: the accent, solid.
  running: { bgcolor: 'running.main', color: 'running.contrastText' },
  // No state: a plain label (the agent, "Expired").
  neutral: { bgcolor: 'grey.100', color: 'grey.800' },
  done: { bgcolor: 'success.tag', color: 'success.text' },
  // Waiting for the operator.
  attention: { bgcolor: 'warning.main', color: 'warning.tagText' },
  failed: { bgcolor: 'error.tint', color: 'error.text' },
};

/** The small status label of the handoff ("Running", "Completed · 18s", "Waiting"). */
const StatusTag = ({ tone = 'neutral', label, sx }) => (
  <Chip size="small" label={label} sx={{ fontWeight: 500, ...TONES[tone], ...sx }} />
);

export default StatusTag;
