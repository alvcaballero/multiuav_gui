import { darken, getContrastRatio } from '@mui/material/styles';
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

/**
 * How close a danger is: the collision sensor (map arcs, sensor widget, the device page alert) and
 * the exclusion zones. A scale of its own, not STATUS: "clear" is not "a task finished well".
 * Text on them goes through `contrastText()`, like on STATUS.
 */
export const HAZARD = {
  /** Nothing within the caution distance. */
  clear: green[500],
  caution: amber[500],
  /** Within the stop distance; an exclusion zone. */
  danger: red[700],
};

/** HAZARD as text on a light background (a distance written in its colour): AA on white. */
export const HAZARD_TEXT = {
  clear: green[800],
  caution: darken(amber[500], 0.5),
  danger: red[700],
};

/**
 * Drawn geometry on the map: geofences, accuracy circles, live routes, mapbox-gl-draw's shapes (it is
 * draw's own default blue). A constant, not only `palette.geometry`, for map/draw/theme.js, which
 * is a static style list with no theme.
 */
export const GEOMETRY = '#3bb2d0';

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
      main: GEOMETRY,
    },
    success: {
      main: colors.positive,
    },
    warning: {
      main: colors.medium,
    },
    status: STATUS,
    hazard: HAZARD,
    colors,
  };
};
