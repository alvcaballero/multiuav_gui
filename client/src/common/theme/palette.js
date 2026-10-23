import { getContrastRatio } from '@mui/material/styles';
import {
  amber,
  blue,
  blueGrey,
  grey,
  green,
  indigo,
  lightBlue,
  red,
  common,
} from '@mui/material/colors';

/**
 * What each colour MEANS wherever a state is shown (missions, tasks, events): the same rule as the
 * assistant panel, whose own theme (components/chat/chatTheme.js) uses its handoff's shades of it.
 * Blue is in progress, green finished well, amber needs the operator or went partly wrong, red
 * failed, grey has not started, blue-grey was stopped before it ended.
 *
 * Hex, not palette paths, so they also work outside `sx`; `palette.status` exposes them to `sx` too.
 * The solid ones (running, done, failed, …) are dark enough for white text at WCAG AA (4.5:1); the
 * light ones take dark text. Never fix the text colour: ask `contrastText(color)`.
 */
export const STATUS = {
  /** In progress: a mission or task running. */
  running: blue[800],
  /** On its way, not yet running: planning, a command sent. */
  pending: blue[300],
  /** Not started: waiting for its dependencies. */
  waiting: grey[400],
  /** Needs the operator, or finished with errors: loaded and waiting for start, an anomaly. */
  attention: amber[700],
  /** Finished well. */
  done: green[800],
  /** Finished, and its files downloaded. */
  archived: green[900],
  failed: red[700],
  cancelled: blueGrey[600],
  /** Never ran: a dependency failed. */
  skipped: blueGrey[300],
  /** An informational event: no state, just news. Not the blue of `running`. */
  info: lightBlue[800],
};

const DARK_TEXT = 'rgba(0, 0, 0, 0.87)';

/** The text colour that reads best on `background`: white or the theme's dark text. */
export const contrastText = (background) =>
  getContrastRatio(background, common.white) >= getContrastRatio(background, DARK_TEXT)
    ? common.white
    : DARK_TEXT;

export default () => {
  const colors = {
    white: common.white,
    background: grey[50],
    primary: indigo[900],
    secondary: green[800],
    positive: green[500],
    medium: amber[700],
    negative: red[500],
    neutral: grey[500],
    geometry: '#3bb2d0',
  };

  const colorsDevices = {
    0: '#F34C28',
    1: '#F39A28',
    2: '#1EC910',
    3: '#1012C9',
    4: '#C310C9',
    5: '#1FDBF1',
    6: '#F6FD04',
    7: '#808080',
  };
  return {
    background: {
      default: colors.background,
    },
    primary: {
      main: colors.primary,
    },
    secondary: {
      main: colors.secondary,
      contrastText: colors.white,
    },
    neutral: {
      main: colors.neutral,
    },
    geometry: {
      main: colors.geometry,
    },
    success: {
      main: colors.positive,
    },
    warning: {
      main: colors.medium,
    },
    status: STATUS,
    colors,
    colorsDevices,
  };
};
