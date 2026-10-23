import { useMemo } from 'react';
import { createTheme } from '@mui/material/styles';
import palette from './palette';
import dimensions from './dimensions';
import components from './components';

/** The options the app theme is built from; the assistant panel builds its own on top of them. */
export const themeOptions = () => ({
  palette: palette(),
  dimensions,
  components,
});

export default () => useMemo(() => createTheme(themeOptions()), []);
