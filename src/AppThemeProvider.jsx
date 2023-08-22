import React from 'react';
import { ThemeProvider } from '@mui/material';
import theme from './common/theme';

const AppThemeProvider = ({ children }) => {

  const themeInstance = theme();

  return (
    <ThemeProvider theme={themeInstance}>
      {children}
    </ThemeProvider>
  );
};

export default AppThemeProvider;
