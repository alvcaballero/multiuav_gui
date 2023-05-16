import React from 'react';
import ReactDOM from 'react-dom/client';
import './index.css';
import App from './App';
import reportWebVitals from './reportWebVitals';
import { Provider } from 'react-redux';
import store from './store';
import preloadImages from './Mapview/preloadImages';
import { CssBaseline, StyledEngineProvider } from '@mui/material';

import AppThemeProvider from './AppThemeProvider';

preloadImages();

const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(
     <Provider store={store}>
        <StyledEngineProvider injectFirst>
          <AppThemeProvider>
            <CssBaseline />
            <App />
          </AppThemeProvider>
        </StyledEngineProvider>

     </Provider>
);

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
reportWebVitals();
