import React, { useState } from 'react';

import { Button, Menu, MenuItem, Fade } from '@mui/material';
import UploadButtons from './uploadButton';

const MenuItems = ({ items, depthLevel }) => {
  const [anchorEl, setAnchorEl] = useState(null);
  const open = Boolean(anchorEl);

  const handleClick = (event) => {
    if (!items.submenu && items.action) {
      items.action();
    }
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  const handleHover = (event) => {
    if (items.submenu) {
      setAnchorEl(event.currentTarget);
    }
  };

  return (
    <div>
      <Button
        id="fade-button"
        aria-controls={open ? 'fade-menu' : undefined}
        aria-haspopup="true"
        aria-expanded={open ? 'true' : undefined}
        onClick={handleClick}
        sx={{ color: '#FFFFFF', fontSize: 16 }}
        style={{ textTransform: 'none' }}
      >
        {items.title}
      </Button>
      {items.submenu && (
        <Menu
          id="fade-menu"
          MenuListProps={{
            'aria-labelledby': 'fade-button',
          }}
          anchorEl={anchorEl}
          open={open}
          onClose={handleClose}
          TransitionComponent={Fade}
        >
          {items.submenu.map((element, index) => {
            if (element.input) {
              return (
                <UploadButtons
                  key={'men' + index}
                  title={element.title}
                  readFile={(e) => {
                    element.input(e);
                  }}
                  typefiles={element.type}
                />
              );
            }
            return (
              <MenuItem
                key={'men' + index}
                onClick={() => {
                  handleClose();
                  if (element.action) element.action();
                }}
              >
                {element.title}
              </MenuItem>
            );
          })}
        </Menu>
      )}
    </div>
  );
};

export default MenuItems;
