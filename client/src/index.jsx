import React, { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { BrowserRouter } from 'react-router-dom';
import { Provider } from 'react-redux';
import { CssBaseline, StyledEngineProvider } from '@mui/material';
import store from './store';
import ErrorHandler from './common/components/ErrorHandler';
import Navigation from './Navigation';
import preloadImages from './Mapview/preloadImages';
import ServerProvider from './ServerProvider';
import AppThemeProvider from './AppThemeProvider';

preloadImages();

const root = createRoot(document.getElementById('root'));
root.render(
  <Provider store={store} stabilityCheck="always">
    <StyledEngineProvider injectFirst>
      <AppThemeProvider>
        <CssBaseline />
        <ServerProvider>
          <BrowserRouter>
            <Navigation />
          </BrowserRouter>
          <ErrorHandler />
        </ServerProvider>
      </AppThemeProvider>
    </StyledEngineProvider>
  </Provider>
);

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
//reportWebVitals();  <StrictMode> </StrictMode>
