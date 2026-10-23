import { Chip } from '@mui/material';

const TONES = {
  // Running: the accent, solid.
  accent: { bgcolor: 'primary.main', color: 'primary.contrastText' },
  neutral: { bgcolor: 'neutral.100', color: 'neutral.800' },
  success: { bgcolor: 'success.tag', color: 'success.text' },
  // Waiting for the operator.
  warning: { bgcolor: 'warning.main', color: 'warning.tagText' },
  error: { bgcolor: 'error.tint', color: 'error.main' },
};

/** The small status label of the handoff ("Running", "Completed · 18s", "Waiting"). */
const StatusTag = ({ tone = 'neutral', label, sx }) => (
  <Chip size="small" label={label} sx={{ fontWeight: 500, ...TONES[tone], ...sx }} />
);

export default StatusTag;
