import { useMemo } from 'react';
import { createTheme } from '@mui/material/styles';
import palette from './palette';
import dimensions from './dimensions';
import components from './components';

export default () => useMemo(() => createTheme({
  palette: palette(),
  dimensions,
  components,
}), []);