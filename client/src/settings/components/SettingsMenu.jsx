import React from 'react';
import { List, ListItemButton, ListItemIcon, ListItemText } from '@mui/material';
import BuildIcon from '@mui/icons-material/Build';
import SmartphoneIcon from '@mui/icons-material/Smartphone';
import CategoryIcon from '@mui/icons-material/Category';
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
      <MenuItem
        title={'element types'}
        link="/settings/elementTypes"
        icon={<CategoryIcon />}
        selected={location.pathname.startsWith('/settings/elementTypes')}
      />
    </List>
  );
};

export default SettingsMenu;
