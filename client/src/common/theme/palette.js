import { darken, getContrastRatio } from '@mui/material/styles';
import {
  amber,
  blue,
  blueGrey,
  grey,
  green,
  indigo,
  lightBlue,
  orange,
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
    /** The dark bar at the very top (Navbar): the app name and its menus. */
    navbar: {
      main: '#333333',
      contrastText: common.white,
    },
    /** The light mission bar under it (layout/Menu.jsx and its menu/ groups). */
    toolbar: {
      /** Its background is a vertical gradient from `top` to `bottom`. */
      top: '#f0f2f5',
      bottom: '#e4e7ec',
      border: '#c8cdd6',
      /** The thin vertical line between groups of buttons. */
      separator: '#c0c5ce',
      shadow: '0 1px 3px rgba(0, 0, 0, 0.08)',
      /** The status box in the middle, lifted off the bar. */
      raisedShadow: '0 1px 2px rgba(0, 0, 0, 0.06)',
      hover: 'rgba(0, 0, 0, 0.07)',
      /** The "something new" dot on an icon (unseen missions, unseen errors). */
      badge: red[500],
    },
    /**
     * The mission buttons of the toolbar. Actions, not states: they do not follow STATUS (Stop is not
     * "failed"). Fly is a gradient from `fly` to `flyEnd`, one shade lighter on hover; the others
     * are an icon in their colour over a pale tint of it on hover.
     */
    missionControl: {
      fly: blue[800],
      flyEnd: blue[900],
      flyHover: blue[700],
      stop: red[700],
      stopHover: red[50],
      pause: orange[800],
      pauseHover: orange[50],
      resume: green[700],
      resumeHover: green[50],
    },
    /** The side panels of the editors (mission, planning): the line under their header bar. */
    panel: {
      headerBorder: '#d4d4d4',
    },
    /** The device a route is assigned to, as a small tag ("unassigned" stays grey). */
    assignedDevice: {
      tint: blue[50],
      main: blue[800],
      border: blue[200],
    },
    colors,
  };
};
