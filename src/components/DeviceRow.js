import React from 'react';
import { useDispatch, useSelector } from 'react-redux';
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
import { fontSize } from '@mui/system';
import useClasses from './useClasses'
//import { useAttributePreference } from '../common/util/preferences';

const styles = theme => ({
  icon: {
    width: '25px',
    height: '25px',
    filter: 'brightness(0) invert(1)',
  },
  batteryText: {
    fontSize: '0.75rem',
    fontWeight: 'normal',
    lineHeight: '0.875rem',
  },
  positive: {
    color: theme.palette.colors.positive,
  },
  medium: {
    color: theme.palette.colors.medium,
  },
  negative: {
    color: theme.palette.colors.negative,
  },
  neutral: {
    color: theme.palette.colors.neutral,
  },
});



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
  const classes = useClasses(styles);
  const dispatch = useDispatch();

  const item = data[index];
  const position = useSelector((state) => state.data.positions[item.id]);



  const devicePrimary = item['name']; //'name';//useAttributePreference('devicePrimary', 'name');
  const deviceSecondary = "test" ;
  const secondaryText = () => {
    let status;
    if (item.status === 'online' || !item.lastUpdate) {
      status = item.status
    } else {
      status = moment(item.lastUpdate).fromNow();
    }
    return (
      <>
        {position && position.attributes.hasOwnProperty('landed_state') && (
        <div style={{fontSize:12}} > {position.attributes.landed_state}</div>)}
        <div style={{fontSize:12}}  className={classes[getStatusColor(item.status)]}>{status}</div>
      </>
    );
  };

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
          secondary={secondaryText()}
          secondaryTypographyProps={{ noWrap: true }}
        />
        {position && (
          <>

            {position.attributes.hasOwnProperty('ignition') && (
              <Tooltip title={`${'positionIgnition'}: ${formatBoolean(position.attributes.ignition)}`}>
                <IconButton size="small">
                  {position.attributes.ignition ? (
                    <EngineIcon width={20} height={20} style={positivecolor} />
                  ) : (
                    <EngineIcon width={20} height={20} style={neutralcolor} />
                  )}
                </IconButton>
              </Tooltip>
            )}
            
            {position.hasOwnProperty('speed') && (
              <Tooltip title={`${'speed'}: ${position.speed}`}>
                <IconButton size="small">
                  <div style={{fontSize:"0.9rem"}}> V {Math.round(position.speed)}m/s</div>
                </IconButton>
              </Tooltip>
            )}
            {position.hasOwnProperty('altitude') && (
              <Tooltip title={`${'altitude'}: ${Math.round(position.altitude)}`}>
                <IconButton size="small">
                  <div style={{fontSize:"0.9rem"}}>H {Math.round(position.altitude)}m</div>
                </IconButton>
              </Tooltip>
            )}
            {position.attributes.hasOwnProperty('batteryLevel') && (
              <Tooltip title={`${'BatteryLevel'}: ${formatPercentage(position.attributes.batteryLevel)}`}>
                <IconButton size="small">
                  {position.attributes.batteryLevel > 70 ? (
                    position.attributes.charge
                      ? (<BatteryChargingFullIcon fontSize="small" style={positivecolor} />)
                      : (<BatteryFullIcon fontSize="small" style={positivecolor}  />)
                  ) : position.attributes.batteryLevel > 30 ? (
                    position.attributes.charge
                      ? (<BatteryCharging60Icon fontSize="small" style={mediumcolor} />)
                      : (<Battery60Icon fontSize="small" style={mediumcolor} />)
                  ) : (
                    position.attributes.charge
                      ? (<BatteryCharging20Icon fontSize="small" style={negativecolor}/>)
                      : (<Battery20Icon fontSize="small" style={negativecolor} />)
                  )}
                </IconButton>
              </Tooltip>
            )}
            {position.attributes.hasOwnProperty('alarm') && (
              <Tooltip title={`${'eventAlarm'}: ${formatAlarm(position.attributes.alarm)}`}>
                <IconButton size="small">
                  <ErrorIcon fontSize="small" style={negativecolor} />
                </IconButton>
              </Tooltip>
            )}
          </>
        )}
      </ListItemButton>
    </div>
  );
};

export default DeviceRow;
