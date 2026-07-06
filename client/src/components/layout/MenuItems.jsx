import React, { useState } from 'react';

import { Button, Menu, MenuItem, Fade } from '@mui/material';
import UploadButtons from '../ui/uploadButton';

const MenuItems = ({ items }) => {
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
          slotProps={{ list: { 'aria-labelledby': 'fade-button' } }}
          anchorEl={anchorEl}
          open={open}
          onClose={handleClose}
          slots={{ transition: Fade }}
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
