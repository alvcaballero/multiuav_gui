// Single source of truth for event severity colors (server `Event.type`: error,
// warning, info, success, or any other custom value).
import { red, orange, blue, green, grey } from '@mui/material/colors';

const EVENT_COLORS = {
  error: red[600],
  warning: orange[700],
  info: blue[600],
  success: green[600],
};

const FALLBACK = grey[500];

export const eventColor = (type) => EVENT_COLORS[type] ?? FALLBACK;
