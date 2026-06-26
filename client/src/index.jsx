import { createRoot } from 'react-dom/client';
import { BrowserRouter } from 'react-router-dom';
import { Provider } from 'react-redux';
import { CssBaseline, StyledEngineProvider } from '@mui/material';
import store from './store';
import ErrorHandler from './shared/components/ErrorHandler';
import Navigation from './Navigation';
import preloadImages from './map/core/preloadImages';
import ServerProvider from './ServerProvider';
import ErrorBoundary from './ErrorBoundary';
import AppThemeProvider from './AppThemeProvider';

// Fire-and-forget: map icons load in background while React mounts
preloadImages();

const root = createRoot(document.getElementById('root'));
root.render(
  <ErrorBoundary>
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
    </Provider>{' '}
  </ErrorBoundary>
);
