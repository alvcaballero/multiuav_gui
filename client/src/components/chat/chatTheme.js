import '@fontsource/barlow/400.css';
import '@fontsource/barlow/500.css';
import '@fontsource/barlow/600.css';
import '@fontsource/barlow-condensed/500.css';
import '@fontsource/barlow-condensed/600.css';
import { createTheme } from '@mui/material/styles';

/**
 * The look of the assistant panel: the "Industry" design system of the handoff mapped onto MUI.
 *
 * It is a theme of its own, given to the panel through a nested ThemeProvider, instead of
 * overrides scattered over `sx` props or a change to the app theme — the rest of the GCS keeps its
 * look and everything under the panel (menus, tooltips and the delete dialog render in portals,
 * which keep the React context) picks this one up.
 *
 * Corners are 4px: `shape.borderRadius` is 4, and MUI's Paper, Button, inputs, Menu and Tooltip
 * all read it. Colour carries the meaning (version 2 of the handoff): green is approved / done,
 * amber is waiting on the operator, red is rejected or failed, teal is a finished subagent, and the
 * accent blue is the operator and anything running.
 */

/** The ramp steps the handoff names; the semantic ones live in the theme palette. */
export const chatColors = {
  bg: '#ffffff',
  surface: '#e9e9ea',
  /** The title bar. */
  titleBar: '#e4e4e6',
  text: '#1d1f20',
  divider: 'rgba(29, 31, 32, 0.16)',
  accent: {
    100: '#eef6ff',
    200: '#d6ebff',
    300: '#b5d9fd',
    400: '#94bce3',
    500: '#749dc4',
    600: '#597ea3',
    700: '#416180',
    800: '#2c455d',
    900: '#1d2d3d',
    main: '#5980a6',
  },
  neutral: {
    100: '#f5f5f8',
    200: '#e7e7ea',
    300: '#d4d4d7',
    700: '#5d5d60',
    800: '#424244',
  },
  /** Approved, done. */
  ok: {
    main: '#2e7d32',
    dark: '#256628',
    text: '#1e5b22',
    tag: '#e3f2e4',
    tint: '#edf7ee',
    border: '#b7dfb9',
  },
  /** Rejected, failed. */
  bad: {
    main: '#c62828',
    border: '#e6a3a3',
    rowBorder: '#f1b8b8',
    tint: '#fdeeee',
  },
  /** Waiting for the operator. */
  warn: {
    main: '#f0b429',
    icon: '#b7791f',
    text: '#8a5a00',
    tagText: '#3d2a00',
    tint: '#fffaf0',
    border: '#f0d58a',
  },
  /** A subagent that finished. */
  teal: {
    main: '#00897b',
    text: '#00796b',
    border: '#80cbc4',
    tint: '#f7fbfb',
    strip: '#e0f2f1',
  },
  /** Hover on the grey title bar and its icon buttons. */
  hover: 'rgba(0, 0, 0, 0.07)',
};

export const shadows = {
  md: '0 3px 10px rgba(43, 43, 45, 0.16)',
  lg: '0 12px 32px rgba(43, 43, 45, 0.22)',
};

export const BODY_FONT = '"Barlow", system-ui, sans-serif';
/** Titles and small-caps labels: 600 weight, uppercase, tracked out. */
export const HEADING_FONT = '"Barlow Condensed", system-ui, sans-serif';
export const MONO_FONT = 'ui-monospace, Menlo, monospace';

/** The small-caps label of the handoff (section kickers, INPUT/OUTPUT, group names). */
export const kickerSx = {
  fontFamily: HEADING_FONT,
  fontSize: 11,
  letterSpacing: '0.12em',
  textTransform: 'uppercase',
  color: chatColors.neutral[700],
  lineHeight: 1.3,
};

/**
 * Off-screen transcript rows are skipped by the browser instead of laid out, styled and painted.
 * `content-visibility: auto` is what buys that; the paired `contain-intrinsic-size` is what makes it
 * safe, because a skipped row would otherwise collapse to zero height and wreck the scrollbar. Its
 * `auto` keyword means the placeholder size is only a guess until the row has been rendered once —
 * after that the browser reuses the real height it remembers, so the scroll position stays put.
 *
 * Preferred over windowing because the rows stay in the DOM: find-in-page, selecting and copying a
 * whole conversation, and the scroll anchoring in `loadOlderMessages` all keep working.
 */
export const OFFSCREEN_SKIP = { contentVisibility: 'auto' };

const focusRing = {
  outline: `2px solid ${chatColors.accent.main}`,
  outlineOffset: 2,
};

export const chatTheme = createTheme({
  shape: { borderRadius: 4 },
  palette: {
    mode: 'light',
    primary: {
      main: chatColors.accent.main,
      dark: chatColors.accent[700],
      contrastText: '#ffffff',
    },
    success: { main: chatColors.ok.main, dark: chatColors.ok.dark, contrastText: '#ffffff' },
    error: { main: chatColors.bad.main },
    warning: { main: chatColors.warn.main, dark: chatColors.warn.text },
    text: { primary: chatColors.text, secondary: chatColors.neutral[700] },
    background: { default: chatColors.bg, paper: chatColors.bg },
    divider: chatColors.divider,
  },
  typography: {
    fontFamily: BODY_FONT,
    fontSize: 14,
    body1: { fontSize: 14, lineHeight: 1.5 },
    body2: { fontSize: 13, lineHeight: 1.45 },
    caption: { fontSize: 12, lineHeight: 1.4 },
    button: { textTransform: 'none', fontWeight: 500, fontSize: 13 },
  },
  components: {
    MuiButtonBase: {
      styleOverrides: { root: { '&.Mui-focusVisible': focusRing } },
    },
    MuiButton: {
      defaultProps: { disableElevation: true },
      styleOverrides: {
        root: { minHeight: 34, paddingInline: 14 },
        sizeSmall: { minHeight: 30, paddingInline: 10 },
        containedPrimary: {
          '&:hover': { backgroundColor: chatColors.accent[600] },
          '&:active': { backgroundColor: chatColors.accent[700] },
        },
        outlinedPrimary: { borderColor: chatColors.divider, color: chatColors.text },
        textPrimary: { color: chatColors.accent[700] },
      },
    },
    MuiIconButton: {
      styleOverrides: { root: { color: chatColors.text } },
    },
    MuiChip: {
      styleOverrides: { root: { height: 22, fontSize: 12, borderRadius: 3 } },
    },
    MuiPaper: {
      styleOverrides: { root: { backgroundImage: 'none' } },
    },
    MuiMenu: {
      styleOverrides: {
        paper: { border: `1px solid ${chatColors.divider}`, boxShadow: shadows.md },
      },
    },
    MuiPopover: {
      styleOverrides: {
        paper: { border: `1px solid ${chatColors.divider}`, boxShadow: shadows.md },
      },
    },
    MuiMenuItem: {
      styleOverrides: { root: { fontSize: 13.5 } },
    },
    MuiOutlinedInput: {
      styleOverrides: {
        notchedOutline: { borderColor: chatColors.divider },
        root: { backgroundColor: chatColors.bg },
      },
    },
    MuiTooltip: {
      styleOverrides: { tooltip: { fontSize: 12 } },
    },
  },
});
