import React, { useState, useRef } from 'react';
import { useSelector } from 'react-redux';

import {
    Toolbar, IconButton, OutlinedInput, InputAdornment, Popover, FormControl, InputLabel, Select, MenuItem, FormGroup, FormControlLabel, Checkbox, Badge, ListItemButton, ListItemText, Tooltip,
  } from '@mui/material';
 
import AddIcon from '@mui/icons-material/Add';
import ViewListIcon from '@mui/icons-material/ViewList';
import TuneIcon from '@mui/icons-material/Tune';

const toolbar= {
    display: 'flex',
    gap: '10px 10px',
    height: '30px',
    borderBottom: "3px solid rgb(212, 212, 212)",
};

const MainToolbar = ({SetAddUAVOpen}) =>{
    
    return (
        <Toolbar style={toolbar}>
            <IconButton edge="start" >
                <ViewListIcon />
            </IconButton>
            <OutlinedInput
            placeholder="device"
            endAdornment={(
                <InputAdornment position="end">
                <IconButton size="small" edge="end" >
                    <Badge color="info" variant="dot" >
                    <TuneIcon fontSize="small" />
                    </Badge>
                </IconButton>
                </InputAdornment>
            )}
            size="small"
            fullWidth
            />
            <IconButton edge="end" onClick={()=>{SetAddUAVOpen(true)}}  >
                <Tooltip  >
                    <AddIcon />
                </Tooltip>
            </IconButton>
        </Toolbar>
    );
};
 export default MainToolbar;