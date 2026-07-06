import React, { useState, useRef } from 'react';
import { useSelector } from 'react-redux';

import {
  Toolbar,
  IconButton,
  OutlinedInput,
  InputAdornment,
  Popover,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  FormGroup,
  FormControlLabel,
  Checkbox,
  Badge,
  ListItemButton,
  ListItemText,
  Tooltip,
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import { useTheme } from '@mui/material/styles';

import AddIcon from '@mui/icons-material/Add';
import ViewListIcon from '@mui/icons-material/ViewList';
import TuneIcon from '@mui/icons-material/Tune';
import DeviceRow from '../devices/DeviceRow';

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

const MainToolbar = React.memo(
  ({
    keyword,
    setKeyword,
    filteredDevices,
    filter,
    setFilter,
    filterSort,
    setFilterSort,
    filterMap,
    setFilterMap,
    SetAddUAVOpen,
  }) => {
    const { classes } = useStyles();
    const theme = useTheme();

    const devices = useSelector((state) => state.devices.items);

    const toolbarRef = useRef();
    const inputRef = useRef();
    const [filterAnchorEl, setFilterAnchorEl] = useState(null);
    const [devicesAnchorEl, setDevicesAnchorEl] = useState(null);

    const deviceStatusCount = (status) =>
      Object.values(devices).filter((d) => d.status === status).length;

    return (
      <Toolbar ref={toolbarRef} className={classes.toolbar}>
        <IconButton edge="start">
          <ViewListIcon />
        </IconButton>
        <OutlinedInput
          ref={inputRef}
          placeholder="device"
          value={keyword}
          onChange={(e) => setKeyword(e.target.value)}
          onFocus={() => setDevicesAnchorEl(toolbarRef.current)}
          onBlur={() => setDevicesAnchorEl(null)}
          endAdornment={
            <InputAdornment position="end">
              <IconButton
                size="small"
                edge="end"
                onClick={() => setFilterAnchorEl(inputRef.current)}
              >
                <Badge color="info" variant="dot" invisible={!filter.statuses.length}>
                  <TuneIcon fontSize="small" />
                </Badge>
              </IconButton>
            </InputAdornment>
          }
          size="small"
          fullWidth
        />
        <Popover
          open={!!devicesAnchorEl}
          anchorEl={devicesAnchorEl}
          onClose={() => setDevicesAnchorEl(null)}
          anchorOrigin={{
            vertical: 'bottom',
            horizontal: Number(theme.spacing(2).slice(0, -2)),
          }}
          marginThreshold={0}
          slotProps={{
            paper: {
              style: { width: `calc(${toolbarRef.current?.clientWidth}px - ${theme.spacing(4)})` },
            },
          }}
          elevation={1}
          disableAutoFocus
          disableEnforceFocus
        >
          {filteredDevices.slice(0, 3).map((_, index) => (
            <DeviceRow key={filteredDevices[index].id} data={filteredDevices} index={index} />
          ))}
        </Popover>
        <Popover
          open={!!filterAnchorEl}
          anchorEl={filterAnchorEl}
          onClose={() => setFilterAnchorEl(null)}
          anchorOrigin={{
            vertical: 'bottom',
            horizontal: 'left',
          }}
        >
          <div className={classes.filterPanel}>
            <FormControl>
              <InputLabel>{'Device status'}</InputLabel>
              <Select
                label={'Device status'}
                value={filter.statuses}
                onChange={(e) => setFilter({ ...filter, statuses: e.target.value })}
                multiple
              >
                <MenuItem value="online">{`${'Online'} (${deviceStatusCount('online')})`}</MenuItem>
                <MenuItem value="offline">{`${'Offline'} (${deviceStatusCount('offline')})`}</MenuItem>
                <MenuItem value="unknown">{`${'Unknown'} (${deviceStatusCount('unknown')})`}</MenuItem>
              </Select>
            </FormControl>
            <FormControl>
              <InputLabel>{'Sort by'}</InputLabel>
              <Select
                label={'Sort by'}
                value={filterSort}
                onChange={(e) => setFilterSort(e.target.value)}
                displayEmpty
              >
                <MenuItem value="">{'\u00a0'}</MenuItem>
                <MenuItem value="name">{'Name'}</MenuItem>
                <MenuItem value="lastUpdate">{'Last update'}</MenuItem>
              </Select>
            </FormControl>
            <FormGroup>
              <FormControlLabel
                control={
                  <Checkbox checked={filterMap} onChange={(e) => setFilterMap(e.target.checked)} />
                }
                label={'Show on map only'}
              />
            </FormGroup>
          </div>
        </Popover>
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
  },
);
export default MainToolbar;
