import { createTheme, darken, lighten } from '@mui/material/styles';
import { grey, teal } from '@mui/material/colors';
import { themeOptions } from '../../common/theme';
import { STATUS, contrastText } from '../../common/theme/palette';

/**
 * The assistant panel's theme: the APP's theme, with the variants the panel needs and its density.
 *
 * It has no colour and no font of its own. Every colour below is derived from the app — its
 * `primary`, its `STATUS`, its toolbar, MUI's grey and teal scales — so when the app changes, the
 * panel follows. The font is the app's (`FONT_FAMILY` in common/theme).
 *
 * What the panel adds is vocabulary, not identity: a state needs a tint behind it, a border, a tag
 * and readable text on each, which the app palette does not spell out. The meaning is the app's:
 * blue `primary` is the operator (their messages, the main actions), `running` is anything in
 * progress (STATUS.running, like a running task), green `success` approved / done, amber `warning`
 * waiting on the operator, red `error` rejected or failed. Only `subagent` (teal, a subagent that
 * finished) has no app counterpart.
 *
 * What it keeps of its own is density: a narrow side panel full of rows, so smaller buttons without
 * capitals, smaller chips and text.
 *
 * It is given to the panel through a nested ThemeProvider. A ThemeProvider given an object REPLACES
 * the outer theme, so this one is built from the app's own options (`themeOptions`): inside the
 * panel `theme.dimensions` and every app palette entry still exist. Built from options, not merged
 * into a finished theme (`createTheme(outer, …)`): that merge happens after MUI has expanded the
 * typography variants and the palette, so sizes would not reach `body1` and the like.
 */

export const MONO_FONT = 'ui-monospace, Menlo, monospace';

/** The small uppercase label of the panel (section kickers, INPUT/OUTPUT, group names). */
export const kickerSx = {
  fontSize: 11,
  fontWeight: 600,
  letterSpacing: '0.12em',
  textTransform: 'uppercase',
  color: 'grey.700',
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

/** A scale around `main`: 100–400 are tints of it (backgrounds, borders), 700–800 darker (text, icons). */
const scale = (main) => ({
  100: lighten(main, 0.92),
  200: lighten(main, 0.84),
  300: lighten(main, 0.7),
  400: lighten(main, 0.5),
  600: main,
  700: darken(main, 0.15),
  800: darken(main, 0.3),
  main,
  contrastText: contrastText(main),
});

/** The variants of a state colour: a tint behind it, a border, a tag background, text on a light ground. */
const tones = (main) => ({
  main,
  contrastText: contrastText(main),
  tint: lighten(main, 0.92),
  tag: lighten(main, 0.85),
  border: lighten(main, 0.55),
  rowBorder: lighten(main, 0.7),
  text: darken(main, 0.2),
});

const app = themeOptions();
const { toolbar } = app.palette;

const attention = tones(STATUS.attention);

export const chatTheme = createTheme({
  ...app,
  palette: {
    ...app.palette,
    // The operator and the main actions: the app's primary, with a scale for tints.
    primary: scale(app.palette.primary.main),
    /** In progress: a tool call, a subagent. STATUS.running, as a running task. */
    running: tones(STATUS.running),
    /** Approved, done. */
    success: tones(STATUS.done),
    /** Rejected, failed. */
    error: tones(STATUS.failed),
    /** Waiting for the operator. Amber is light: its text and icon are darkened to read on white. */
    warning: {
      ...attention,
      dark: darken(STATUS.attention, 0.5),
      text: darken(STATUS.attention, 0.5),
      icon: darken(STATUS.attention, 0.25),
      tagText: contrastText(STATUS.attention),
    },
    /** A subagent that finished. The one state the app has no colour for: MUI's teal. */
    subagent: {
      main: teal[600],
      text: teal[700],
      border: teal[200],
      tint: lighten(teal[600], 0.96),
      strip: teal[50],
    },
    background: {
      ...app.palette.background,
      /** Code blocks, tables, quoted tasks. */
      surface: grey[200],
    },
    /** The panel's title bar: the app's toolbar colours. */
    titleBar: {
      main: toolbar.bottom,
      hover: toolbar.hover,
      border: toolbar.border,
      control: toolbar.border,
    },
  },
  typography: {
    ...app.typography,
    fontSize: 14,
    body1: { fontSize: 14, lineHeight: 1.5 },
    body2: { fontSize: 13, lineHeight: 1.45 },
    caption: { fontSize: 12, lineHeight: 1.4 },
    button: { textTransform: 'none', fontWeight: 500, fontSize: 13 },
  },
  // Per component, the panel's entry replaces the app's (its 40px buttons, its outlined-input
  // background, its 500ms tooltip delay); the app's others (Snackbar, ...) still apply. Colours
  // come from the theme being built, through the style callbacks.
  components: {
    ...app.components,
    MuiButtonBase: {
      styleOverrides: {
        root: ({ theme }) => ({
          '&.Mui-focusVisible': {
            outline: `2px solid ${theme.palette.primary.main}`,
            outlineOffset: 2,
          },
        }),
      },
    },
    MuiButton: {
      defaultProps: { disableElevation: true },
      styleOverrides: {
        root: { minHeight: 34, paddingInline: 14 },
        sizeSmall: { minHeight: 30, paddingInline: 10 },
        // A secondary action reads as neutral, not as a second primary.
        outlinedPrimary: ({ theme }) => ({
          borderColor: theme.palette.divider,
          color: theme.palette.text.primary,
        }),
      },
    },
    MuiIconButton: {
      styleOverrides: { root: ({ theme }) => ({ color: theme.palette.text.primary }) },
    },
    MuiChip: {
      styleOverrides: { root: { height: 22, fontSize: 12, borderRadius: 3 } },
    },
    MuiPaper: {
      styleOverrides: { root: { backgroundImage: 'none' } },
    },
    MuiMenu: {
      styleOverrides: {
        paper: ({ theme }) => ({
          border: `1px solid ${theme.palette.divider}`,
          boxShadow: theme.shadows[4],
        }),
      },
    },
    MuiPopover: {
      styleOverrides: {
        paper: ({ theme }) => ({
          border: `1px solid ${theme.palette.divider}`,
          boxShadow: theme.shadows[4],
        }),
      },
    },
    MuiMenuItem: {
      styleOverrides: { root: { fontSize: 13.5 } },
    },
    // The app makes every form control small; the panel's inputs keep their size.
    MuiFormControl: {
      defaultProps: { size: 'medium' },
    },
    MuiOutlinedInput: {
      styleOverrides: {
        notchedOutline: ({ theme }) => ({ borderColor: theme.palette.divider }),
        root: ({ theme }) => ({ backgroundColor: theme.palette.background.paper }),
      },
    },
    MuiTooltip: {
      styleOverrides: { tooltip: { fontSize: 12 } },
    },
  },
});
