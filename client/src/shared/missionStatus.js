// Single source of truth for mission/task status colors + labels.
//
// Two DISTINCT status axes — do not merge them. `running` means different
// things in each, so each axis has its own map. Server enum: server/config/status.js
//   MISSION_STATUS: init, planning, running, finish, finish_errors, done, cancelled, error
//   TASK_STATUS:    init, loaded, commanded, running, complete, end, cancelled, error, skipped
//
// Colors are HEX from the MUI palette (not MUI color names like 'success') so
// every consumer renders the exact same color via sx={{ backgroundColor, color }}.
import { blue, green, grey, amber, red, blueGrey } from '@mui/material/colors';

const MISSION_STATUS_STYLE = {
  init: { color: blue[600], label: 'Init' },
  planning: { color: blue[300], label: 'Planning' },
  running: { color: green[700], label: 'Running' },
  finish: { color: grey[500], label: 'Done' }, // MISSION_STATUS.COMPLETED
  finish_errors: { color: amber[700], label: 'Done (errors)' }, // MISSION_STATUS.COMPLETED_WITH_ERRORS
  done: { color: grey[600], label: 'Downloaded' }, // MISSION_STATUS.END (finished + files downloaded)
  error: { color: red[700], label: 'Error' },
  cancelled: { color: blueGrey[600], label: 'Cancelled' },
};

const TASK_STATUS_STYLE = {
  init: { color: grey[400], label: 'Waiting' }, // created, waiting for its dependencies
  loaded: { color: amber[600], label: 'Loaded' },
  commanded: { color: blue[600], label: 'Sent' },
  running: { color: green[700], label: 'Running' },
  complete: { color: grey[500], label: 'Done' },
  end: { color: grey[600], label: 'Downloaded' },
  cancelled: { color: blueGrey[600], label: 'Cancelled' },
  error: { color: red[700], label: 'Error' },
  skipped: { color: blueGrey[300], label: 'Skipped' }, // never ran: a dependency failed
};

const FALLBACK = (s) => ({ color: grey[400], label: s ?? 'unknown' });

export const missionStyle = (s) => MISSION_STATUS_STYLE[s] ?? FALLBACK(s);
export const taskStyle = (s) => TASK_STATUS_STYLE[s] ?? FALLBACK(s);
