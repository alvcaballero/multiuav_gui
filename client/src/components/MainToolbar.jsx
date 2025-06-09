import React, { useState, useRef } from 'react';
import { useSelector } from 'react-redux';

import { Toolbar, IconButton, OutlinedInput, InputAdornment, Badge, Tooltip } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import { useTheme } from '@mui/material/styles';

import AddIcon from '@mui/icons-material/Add';
import ViewListIcon from '@mui/icons-material/ViewList';
import TuneIcon from '@mui/icons-material/Tune';

const toolbar = {
  display: 'flex',
  gap: '10px 10px',
  height: '30px',
  borderBottom: '3px solid rgb(212, 212, 212)',
};

const useStyles = makeStyles()((theme) => ({
  toolbar: {
    display: 'flex',
    gap: theme.spacing(1),
  },
  filterPanel: {
    display: 'flex',
    flexDirection: 'column',
    padding: theme.spacing(2),
    gap: theme.spacing(2),
    width: theme.dimensions.drawerWidthTablet,
  },
}));

const MainToolbar = React.memo(({ SetAddUAVOpen }) => {
  const { classes } = useStyles();
  const theme = useTheme();

  const toolbarRef = useRef();
  const inputRef = useRef();
  const [filterAnchorEl, setFilterAnchorEl] = useState(null);
  const [devicesAnchorEl, setDevicesAnchorEl] = useState(null);
  return (
    <Toolbar ref={toolbarRef} className={classes.toolbar}>
      <IconButton edge="start">
        <ViewListIcon />
      </IconButton>
      <OutlinedInput
        ref={inputRef}
        placeholder="device"
        endAdornment={
          <InputAdornment position="end">
            <IconButton size="small" edge="end">
              <Badge color="info" variant="dot">
                <TuneIcon fontSize="small" />
              </Badge>
            </IconButton>
          </InputAdornment>
        }
        size="small"
        fullWidth
      />
      <IconButton
        edge="end"
        onClick={() => {
          SetAddUAVOpen(true);
        }}
      >
        <Tooltip>
          <AddIcon />
        </Tooltip>
      </IconButton>
    </Toolbar>
  );
});
export default MainToolbar;
