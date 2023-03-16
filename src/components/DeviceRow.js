import React from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { makeStyles } from 'tss-react/mui';
import {
  IconButton, Tooltip, Avatar, ListItemAvatar, ListItemText, ListItemButton,
} from '@mui/material';
import {
    amber, grey, green, indigo, red, common,
  } from '@mui/material/colors';
import BatteryFullIcon from '@mui/icons-material/BatteryFull';
import BatteryChargingFullIcon from '@mui/icons-material/BatteryChargingFull';
import Battery60Icon from '@mui/icons-material/Battery60';
import BatteryCharging60Icon from '@mui/icons-material/BatteryCharging60';
import Battery20Icon from '@mui/icons-material/Battery20';
import BatteryCharging20Icon from '@mui/icons-material/BatteryCharging20';
import ErrorIcon from '@mui/icons-material/Error';
import moment from 'moment';
import { devicesActions } from '../store';
import { formatAlarm, formatBoolean, formatPercentage, formatStatus, getStatusColor,} from '../common/formatter';
import { mapIconKey, mapIcons } from '../Mapview/preloadImages';
import { ReactComponent as EngineIcon } from '../resources/images/data/engine.svg';
//import { useAttributePreference } from '../common/util/preferences';


  const iconStyle= {
    width: '25px',
    height: '25px',
    filter: 'brightness(0) invert(1)',
  };
  const batteryTextStyle= {
    fontSize: '0.75rem',
    fontWeight: 'normal',
    lineHeight: '0.875rem',
  };
  const positivecolor={
    color: green[500],
  };
  const mediumcolor= {
    color: amber[700],
  };
  const negativecolor= {
    color: red[500],
  };
  const neutralcolor= {
    color: grey[500],
  };


const DeviceRow = ({ data, index, style }) => {
  const dispatch = useDispatch();

  const item = data[index];



  const devicePrimary = item['name']; //'name';//useAttributePreference('devicePrimary', 'name');

  return (
    <div style={style}>
      <ListItemButton
        key={item.id}
        onClick={() => dispatch(devicesActions.selectId(item.id))}
        disabled={ item.disabled}
      >
        <ListItemAvatar>
          <Avatar>
            <img style={iconStyle} src={mapIcons[mapIconKey(item.category)]} alt="" />
          </Avatar>
        </ListItemAvatar>
        <ListItemText
          primary={devicePrimary}
          primaryTypographyProps={{ noWrap: true }}
        />
      </ListItemButton>
    </div>
  );
};

export default DeviceRow;
