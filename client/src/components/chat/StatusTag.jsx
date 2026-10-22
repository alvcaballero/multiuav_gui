import { Chip } from '@mui/material';
import { chatColors } from './chatTheme';

const TONES = {
  // Running: the accent, solid.
  accent: { bgcolor: chatColors.accent.main, color: '#ffffff' },
  neutral: { bgcolor: chatColors.neutral[100], color: chatColors.neutral[800] },
  success: { bgcolor: chatColors.ok.tag, color: chatColors.ok.text },
  // Waiting for the operator.
  warning: { bgcolor: chatColors.warn.main, color: chatColors.warn.tagText },
  error: { bgcolor: chatColors.bad.tint, color: chatColors.bad.main },
};

/** The small status label of the handoff ("Running", "Completed · 18s", "Waiting"). */
const StatusTag = ({ tone = 'neutral', label, sx }) => (
  <Chip size="small" label={label} sx={{ fontWeight: 500, ...TONES[tone], ...sx }} />
);

export default StatusTag;
