// Single source of truth for event severity colors (server `Event.type`: error,
// warning, info, success, or any other custom value). Colors from the theme's STATUS
// (common/theme/palette.js), so a red event and a red task mean the same thing.
import { STATUS } from '../../theme/palette';

const EVENT_COLORS = {
  error: STATUS.failed,
  warning: STATUS.attention,
  info: STATUS.info,
  success: STATUS.done,
};

const FALLBACK = STATUS.waiting;

export const eventColor = (type) => EVENT_COLORS[type] ?? FALLBACK;
