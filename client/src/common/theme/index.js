import { useMemo } from 'react';
import { createTheme } from '@mui/material/styles';
import palette from './palette';
import dimensions from './dimensions';
import components from './components';

/**
 * The system font stack: no web font is loaded (a GCS can be offline), so each machine shows the
 * first of these it has. MapLibre does not use it: map labels are drawn from its own glyph files.
 */
export const FONT_FAMILY = 'Roboto, "Segoe UI", "Helvetica Neue", Arial, sans-serif';

/** The options the app theme is built from; the assistant panel builds its own on top of them. */
export const themeOptions = () => ({
  typography: {
    fontFamily: FONT_FAMILY,
  },
  palette: palette(),
  dimensions,
  components,
});

export default () => useMemo(() => createTheme(themeOptions()), []);
