import React from 'react';
import { Divider, List, ListItemButton, ListItemIcon, ListItemText } from '@mui/material';
import SettingsIcon from '@mui/icons-material/Settings';
import CreateIcon from '@mui/icons-material/Create';
import NotificationsIcon from '@mui/icons-material/Notifications';
import FolderIcon from '@mui/icons-material/Folder';
import PersonIcon from '@mui/icons-material/Person';
import StorageIcon from '@mui/icons-material/Storage';
import BuildIcon from '@mui/icons-material/Build';
import PeopleIcon from '@mui/icons-material/People';
import TodayIcon from '@mui/icons-material/Today';
import PublishIcon from '@mui/icons-material/Publish';
import SmartphoneIcon from '@mui/icons-material/Smartphone';
import HelpIcon from '@mui/icons-material/Help';
import CampaignIcon from '@mui/icons-material/Campaign';
import { Link, useLocation } from 'react-router-dom';

const MenuItem = ({ title, link, icon, selected }) => (
  <ListItemButton key={link} component={Link} to={link} selected={selected}>
    <ListItemIcon>{icon}</ListItemIcon>
    <ListItemText primary={title} />
  </ListItemButton>
);

const SettingsMenu = () => {
  const location = useLocation();

  return (
    <List>
      <MenuItem
        title={'devices'}
        link="/settings/devices"
        icon={<SmartphoneIcon />}
        selected={location.pathname === '/settings/devices'}
      />
      <MenuItem
        title={'category'}
        link="/settings/category"
        icon={<BuildIcon />}
        selected={location.pathname === '/settings/category'}
      />
    </List>
  );
};

export default SettingsMenu;
