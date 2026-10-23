import '@fontsource/barlow/400.css';
import '@fontsource/barlow/500.css';
import '@fontsource/barlow/600.css';
import '@fontsource/barlow-condensed/500.css';
import '@fontsource/barlow-condensed/600.css';
import { createTheme } from '@mui/material/styles';
import { themeOptions } from '../../common/theme';

/**
 * The look of the assistant panel: the "Industry" design system of the handoff mapped onto MUI.
 *
 * It is given to the panel through a nested ThemeProvider. A ThemeProvider given an object REPLACES
 * the outer theme, so this one is built from the app's own options (`themeOptions`) with the
 * handoff's on top: inside the panel `theme.dimensions`, `palette.colors` and `palette.colorsDevices`
 * still exist, and any common/ component rendered there works. Menus, tooltips and dialogs render in
 * portals, which keep the React context, so they pick it up too.
 *
 * Built from options, not merged into a finished theme (`createTheme(outer, …)`): that merge happens
 * after MUI has expanded the typography variants and the palette, so the font would not reach
 * `body1` and the like, and `light`/`dark`/`contrastText` would keep the app's values.
 *
 * Every colour lives in the palette below: components ask for it by path in `sx`
 * (`bgcolor: 'warning.tint'`, `color: 'primary.700'`) and never import a hex value. Colour carries
 * the meaning (version 2 of the handoff): green (success) is approved / done, amber (warning) is
 * waiting on the operator, red (error) is rejected or failed, teal (subagent) is a finished
 * subagent, and the accent blue (primary) is the operator and anything running.
 *
 * Corners are 4px: `shape.borderRadius` is 4, and MUI's Paper, Button, inputs, Menu and Tooltip
 * all read it.
 */

const accent = {
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
};

const divider = 'rgba(29, 31, 32, 0.16)';
const text = '#1d1f20';

export const shadows = {
  md: '0 3px 10px rgba(43, 43, 45, 0.16)',
  lg: '0 12px 32px rgba(43, 43, 45, 0.22)',
};

/**
 * The frame of the docked panel. ChatDrawer draws it outside the panel's ThemeProvider (around the
 * resizable box), so it cannot ask the palette for the divider.
 */
export const DOCKED_FRAME = {
  borderLeft: `1px solid ${divider}`,
  boxShadow: shadows.lg,
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
  color: 'neutral.700',
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
  outline: `2px solid ${accent.main}`,
  outlineOffset: 2,
};

const app = themeOptions();

export const chatTheme = createTheme({
  ...app,
  shape: { borderRadius: 4 },
  palette: {
    ...app.palette,
    mode: 'light',
    // The accent ramp: `primary.100` … `primary.900`.
    primary: { ...accent, dark: accent[700], contrastText: '#ffffff' },
    /** Approved, done. */
    success: {
      main: '#2e7d32',
      dark: '#256628',
      contrastText: '#ffffff',
      text: '#1e5b22',
      tag: '#e3f2e4',
      tint: '#edf7ee',
      border: '#b7dfb9',
    },
    /** Rejected, failed. */
    error: { main: '#c62828', border: '#e6a3a3', rowBorder: '#f1b8b8', tint: '#fdeeee' },
    /** Waiting for the operator. */
    warning: {
      main: '#f0b429',
      dark: '#8a5a00',
      icon: '#b7791f',
      text: '#8a5a00',
      tagText: '#3d2a00',
      tint: '#fffaf0',
      border: '#f0d58a',
    },
    /** A subagent that finished. */
    subagent: {
      main: '#00897b',
      text: '#00796b',
      border: '#80cbc4',
      tint: '#f7fbfb',
      strip: '#e0f2f1',
    },
    // The app's `neutral` (main) plus the steps the handoff names.
    neutral: {
      ...app.palette.neutral,
      100: '#f5f5f8',
      200: '#e7e7ea',
      300: '#d4d4d7',
      700: '#5d5d60',
      800: '#424244',
    },
    text: { primary: text, secondary: '#5d5d60' },
    background: {
      default: '#ffffff',
      paper: '#ffffff',
      /** Code blocks, tables, quoted tasks. */
      surface: '#e9e9ea',
    },
    /** The grey title bar: its fill, the hover of its buttons, its bottom line, the switcher's outline. */
    titleBar: {
      main: '#e4e4e6',
      hover: 'rgba(0, 0, 0, 0.07)',
      border: 'rgba(0, 0, 0, 0.12)',
      control: 'rgba(0, 0, 0, 0.18)',
    },
    divider,
  },
  typography: {
    fontFamily: BODY_FONT,
    fontSize: 14,
    body1: { fontSize: 14, lineHeight: 1.5 },
    body2: { fontSize: 13, lineHeight: 1.45 },
    caption: { fontSize: 12, lineHeight: 1.4 },
    button: { textTransform: 'none', fontWeight: 500, fontSize: 13 },
  },
  // Per component, the panel's entry replaces the app's (its 40px buttons, its outlined-input
  // background, its 500ms tooltip delay); the app's others (Snackbar, ...) still apply.
  components: {
    ...app.components,
    MuiButtonBase: {
      styleOverrides: { root: { '&.Mui-focusVisible': focusRing } },
    },
    MuiButton: {
      defaultProps: { disableElevation: true },
      styleOverrides: {
        root: { minHeight: 34, paddingInline: 14 },
        sizeSmall: { minHeight: 30, paddingInline: 10 },
        containedPrimary: {
          '&:hover': { backgroundColor: accent[600] },
          '&:active': { backgroundColor: accent[700] },
        },
        outlinedPrimary: { borderColor: divider, color: text },
        textPrimary: { color: accent[700] },
      },
    },
    MuiIconButton: {
      styleOverrides: { root: { color: text } },
    },
    MuiChip: {
      styleOverrides: { root: { height: 22, fontSize: 12, borderRadius: 3 } },
    },
    MuiPaper: {
      styleOverrides: { root: { backgroundImage: 'none' } },
    },
    MuiMenu: {
      styleOverrides: {
        paper: { border: `1px solid ${divider}`, boxShadow: shadows.md },
      },
    },
    MuiPopover: {
      styleOverrides: {
        paper: { border: `1px solid ${divider}`, boxShadow: shadows.md },
      },
    },
    MuiMenuItem: {
      styleOverrides: { root: { fontSize: 13.5 } },
    },
    // The app makes every form control small; the panel's inputs are sized by the handoff.
    MuiFormControl: {
      defaultProps: { size: 'medium' },
    },
    MuiOutlinedInput: {
      styleOverrides: {
        notchedOutline: { borderColor: divider },
        root: { backgroundColor: '#ffffff' },
      },
    },
    MuiTooltip: {
      styleOverrides: { tooltip: { fontSize: 12 } },
    },
  },
});
