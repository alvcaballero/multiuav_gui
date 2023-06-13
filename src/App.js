import './App.css';
import React from 'react';
import { Outlet, useNavigate } from 'react-router-dom';
import makeStyles from '@mui/styles/makeStyles';
import { LinearProgress, useMediaQuery, useTheme } from '@mui/material';
import SocketController from './SocketController';

const useStyles = makeStyles(() => ({
  page: {
    flexGrow: 1,
    overflow: 'auto',
  },
  menu: {
    zIndex: 4,
  },
}));

function App() {
  const classes = useStyles();
  const theme = useTheme();

  return (
  <>
    <SocketController/>
    <div className={classes.page}>
          <Outlet />
    </div>
  </>
  );
};

export default App;
