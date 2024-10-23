import React, { useContext, useState, Fragment, useEffect, useRef } from 'react';
import { useNavigate } from 'react-router-dom';

import {
  AppBar,
  IconButton,
  Toolbar,
  Tooltip,
  Typography,
  Container,
  Button,
  Menu,
  MenuItem,
  Fade,
} from '@mui/material';
import { makeStyles } from '@mui/styles';
import AccountCircleIcon from '@mui/icons-material/AccountCircle';

import { WbIncandescent, WbIncandescentOutlined, Wifi, WifiOff } from '@mui/icons-material';
import Logo from '../resources/images/grvc.svg?react';
import MenuItems from './MenuItems';

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
  const navigate = useNavigate();

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

  const menuItemsData = [
    {
      title: 'ROS',
      url: '/ros',
      submenu: [
        {
          title: 'Connect ROS',
          action: () => navigate('/topics'),
        },
        {
          title: 'Show Topics',
          action: () => navigate('/topics'),
        },
        {
          title: 'Show Services',
          url: 'seo',
          input: 'yaml',
        },
      ],
    },
    {
      title: 'uav',
      url: '/services',
      submenu: [
        {
          title: 'Web Design',
          url: 'web-design',
        },
        {
          title: 'Web Development',
          url: 'web-dev',
        },
        {
          title: 'SEO',
          url: 'seo',
        },
      ],
    },
  ];

  return (
    <AppBar position="static" color="transparent" className={classes.toolbar}>
      <Container maxWidth="xl">
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

          {React.Children.toArray(
            menuItemsData.map((menu, index) => (
              <Fragment key={'s1-' + index}>
                <MenuItems items={menu} depthLevel={0} />
              </Fragment>
            ))
          )}
          {darkModeButton}
          {connectionIndicator}
          <AccountCircleIcon />
        </Toolbar>
      </Container>
    </AppBar>
  );
};
