import { AppBar, IconButton, Toolbar, Tooltip, Typography } from '@mui/material';
import { makeStyles, createStyles } from '@mui/styles';

import { WbIncandescent, WbIncandescentOutlined, Wifi, WifiOff } from '@mui/icons-material';
import React, { ReactElement, useContext, useState } from 'react';
//import AppContext from '../contexts/AppContext';
//import WebSocketContext from '../contexts/WebSocketContext';
import Logo from '../resources/images/grvc.svg?react';
import AccountCircleIcon from '@mui/icons-material/AccountCircle';
//import UserProfileButton from './UserProfileButton';

const useStyles = makeStyles((theme) => ({
  toolbar: {
    backgroundColor: theme.palette.background.paper,
  },
  image: {
    alignSelf: 'center',
    maxWidth: '240px',
    maxHeight: '20px',
    width: 'auto',
    height: 'auto',
  },
}));

export const Navbar2 = ({ title, navIcon, tabs }) => {
  // const ws = useContext(WebSocketContext);
  // const appContext = useContext(AppContext);
  // const { darkMode, setDarkMode } = appContext;
  const [ws, setws] = useState(null);
  const [darkMode, setDarkMode] = useState(false);
  const classes = useStyles();

  const darkModeText = darkMode ? 'Use Light Mode' : 'Use Dark Mode';
  const darkModeButton = (
    <Tooltip arrow title={darkModeText}>
      <IconButton onClick={() => setDarkMode((darkMode) => !darkMode)}>
        {darkMode ? <WbIncandescent /> : <WbIncandescentOutlined />}
      </IconButton>
    </Tooltip>
  );

  const indicateConnected = <Wifi />;
  const indicateDisconnected = <WifiOff color="disabled" />;
  const connectionIndicator = (
    <Tooltip arrow title={ws?.ws?.url ?? 'Server unknown'}>
      <IconButton>{ws?.connected ? indicateConnected : indicateDisconnected}</IconButton>
    </Tooltip>
  );

  return (
    <AppBar position="static" color="transparent" className={classes.toolbar}>
      <Toolbar variant="dense">
        {navIcon ?? (
          <IconButton>
            <Logo className={classes.image} />
          </IconButton>
        )}
        <Typography variant="h6" style={{ flexGrow: 1, marginLeft: '1rem' }}>
          {title ?? 'Management Tool'}
        </Typography>
        {tabs && <>{tabs}</>}
        {darkModeButton}
        {connectionIndicator}
        <AccountCircleIcon />
      </Toolbar>
    </AppBar>
  );
};
