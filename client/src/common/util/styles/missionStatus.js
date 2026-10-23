// Single source of truth for mission/task status colors + labels.
//
// Two DISTINCT status axes — do not merge them. `running` means different
// things in each, so each axis has its own map. Server enum: server/config/status.js
//   MISSION_STATUS: init, planning, running, finish, finish_errors, done, cancelled, error
//   TASK_STATUS:    init, loaded, commanded, running, complete, end, cancelled, error, skipped
//
// Colors come from the theme's STATUS (common/theme/palette.js), which says what each one means:
// blue in progress, green finished well, amber needs the operator, red failed. They are HEX so
// every consumer renders the exact same color, in sx={{ backgroundColor, color }} or anywhere else.
import { STATUS, contrastText } from '../../theme/palette';

const MISSION_STATUS_STYLE = {
  init: { color: STATUS.waiting, label: 'Init' },
  planning: { color: STATUS.pending, label: 'Planning' },
  running: { color: STATUS.running, label: 'Running' },
  finish: { color: STATUS.done, label: 'Done' }, // MISSION_STATUS.COMPLETED
  finish_errors: { color: STATUS.attention, label: 'Done (errors)' }, // MISSION_STATUS.COMPLETED_WITH_ERRORS
  done: { color: STATUS.archived, label: 'Downloaded' }, // MISSION_STATUS.END (finished + files downloaded)
  error: { color: STATUS.failed, label: 'Error' },
  cancelled: { color: STATUS.cancelled, label: 'Cancelled' },
};

const TASK_STATUS_STYLE = {
  init: { color: STATUS.waiting, label: 'Waiting' }, // created, waiting for its dependencies
  loaded: { color: STATUS.attention, label: 'Loaded' }, // on the UAV, waiting for the start command
  commanded: { color: STATUS.pending, label: 'Sent' },
  running: { color: STATUS.running, label: 'Running' },
  complete: { color: STATUS.done, label: 'Done' },
  end: { color: STATUS.archived, label: 'Downloaded' },
  cancelled: { color: STATUS.cancelled, label: 'Cancelled' },
  error: { color: STATUS.failed, label: 'Error' },
  skipped: { color: STATUS.skipped, label: 'Skipped' }, // never ran: a dependency failed
};

const FALLBACK = (s) => ({ color: STATUS.waiting, label: s ?? 'unknown' });

// `textColor` is the text that reads on `color` (white on the solid ones, dark on the light ones).
const withText = (style) => ({ ...style, textColor: contrastText(style.color) });

export const missionStyle = (s) => withText(MISSION_STATUS_STYLE[s] ?? FALLBACK(s));
export const taskStyle = (s) => withText(TASK_STATUS_STYLE[s] ?? FALLBACK(s));
