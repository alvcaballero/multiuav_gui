import React, { useCallback, useMemo } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { IconButton, Tooltip, Avatar, ListItemAvatar, ListItemText, ListItemButton, Typography } from '@mui/material';
import BatteryFullIcon from '@mui/icons-material/BatteryFull';
import BatteryChargingFullIcon from '@mui/icons-material/BatteryChargingFull';
import Battery60Icon from '@mui/icons-material/Battery60';
import BatteryCharging60Icon from '@mui/icons-material/BatteryCharging60';
import Battery20Icon from '@mui/icons-material/Battery20';
import BatteryCharging20Icon from '@mui/icons-material/BatteryCharging20';
import ErrorIcon from '@mui/icons-material/Error';
import dayjs from 'dayjs';
import relativeTime from 'dayjs/plugin/relativeTime';
import { devicesActions } from '../store';
import { formatAlarm, formatBoolean, formatPercentage, formatStatus, getStatusColor } from '../common/formatter';
import { mapIconKey, mapIcons } from '../Mapview/preloadImages';
import EngineIcon from '../resources/images/data/engine.svg';
import { makeStyles } from 'tss-react/mui';

dayjs.extend(relativeTime);

const useStyles = makeStyles()((theme) => ({
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
  success: {
    color: theme.palette.success.main,
  },
  warning: {
    color: theme.palette.warning.main,
  },
  error: {
    color: theme.palette.error.main,
  },
  neutral: {
    color: theme.palette.neutral.main,
  },
}));

const PositionAlarm = React.memo(({ alarm, classes }) =>
  alarm === 'threat' ? (
    <Tooltip title={`${'eventAlarm'}: ${formatAlarm(alarm)}`}>
      <IconButton size="small">
        <ErrorIcon fontSize="small" className={classes.error} />
      </IconButton>
    </Tooltip>
  ) : null
);

const PositionIgnition = React.memo(({ ignition, classes }) => (
  <Tooltip title={`${'positionIgnition'}: ${formatBoolean(ignition)}`}>
    <IconButton size="small">
      {ignition ? (
        <EngineIcon width={20} height={20} style={classes.success} />
      ) : (
        <EngineIcon width={20} height={20} style={classes.neutral} />
      )}
    </IconButton>
  </Tooltip>
));

const PositionSpeed = React.memo(({ speed }) => (
  <Tooltip title={`${'speed'}: ${speed}`}>
    <IconButton size="small">
      <div style={{ fontSize: '0.9rem' }}>V {speed}m/s</div>
    </IconButton>
  </Tooltip>
));

const PositionAltitude = React.memo(({ altitude }) => (
  <Tooltip title={`${'altitude'}: ${Math.round(altitude)}`}>
    <IconButton size="small">
      <div style={{ fontSize: '0.9rem' }}>H {altitude.toFixed(1)}m</div>
    </IconButton>
  </Tooltip>
));

const PositionBattery = React.memo(({ batteryLevel, charge, classes }) => {
  const getBatteryIcon = (level, isCharging) => {
    if (level > 70) {
      return isCharging ? (
        <BatteryChargingFullIcon fontSize="small" className={classes.success} />
      ) : (
        <BatteryFullIcon fontSize="small" className={classes.success} />
      );
    }
    if (level > 30) {
      return isCharging ? (
        <BatteryCharging60Icon fontSize="small" className={classes.warning} />
      ) : (
        <Battery60Icon fontSize="small" className={classes.warning} />
      );
    }
    return isCharging ? (
      <BatteryCharging20Icon fontSize="small" className={classes.error} />
    ) : (
      <Battery20Icon fontSize="small" className={classes.error} />
    );
  };

  return (
    <Tooltip title={`${'BatteryLevel'}: ${formatPercentage(batteryLevel)}`}>
      <IconButton size="small">{getBatteryIcon(batteryLevel, charge)}</IconButton>
    </Tooltip>
  );
});

const customEqual = (oldValue, newValue) => {
  return (
    oldValue?.attributes?.landed_state === newValue?.attributes?.landed_state &&
    oldValue?.attributes?.ignition === newValue?.attributes?.ignition &&
    oldValue?.attributes?.batteryLevel === newValue?.attributes?.batteryLevel &&
    oldValue?.attributes?.alarm === newValue?.attributes?.alarm &&
    oldValue?.attributes?.charge === newValue?.attributes?.charge &&
    oldValue?.speed === newValue?.speed &&
    oldValue?.altitude === newValue?.altitude
  );
};

const DeviceRow = ({ data, index, style }) => {
  const { classes } = useStyles();
  const dispatch = useDispatch();

  const item = data[index];
  const position = useSelector((state) => state.session.positions[item.id], customEqual);

  const devicePrimary = item['name'];
  const deviceSecondary = 'test';

  const secondaryText = useCallback(() => {
    let status;
    if (item.status === 'online') {
      status = item.status;
    } else {
      status = dayjs(item.lastUpdate).fromNow();
    }
    let uavStatus = null;

    if (position?.attributes?.navState) {
      uavStatus = (
        <Typography component="span" variant="body2" sx={{ display: 'block', fontSize: 10 }}>
          {position.attributes.armState} - {position.attributes.navState}
        </Typography>
      );
    } else if (position?.attributes?.landed_state) {
      uavStatus = (
        <Typography component="span" variant="body2" sx={{ display: 'block', fontSize: 10 }}>
          {position.attributes.landed_state}
        </Typography>
      );
    }

    return (
      <>
        {uavStatus}
        <Typography component="span" style={{ fontSize: 12 }} className={classes[getStatusColor(item.status)]}>
          {status}
        </Typography>
      </>
    );
  }, [item.status, item.lastUpdate, position?.attributes?.landed_state, classes]); // Dependencias de useCallback

  return (
    <div style={style}>
      <ListItemButton key={item.id} onClick={() => dispatch(devicesActions.selectId(item.id))} disabled={item.disabled}>
        <ListItemAvatar>
          <Avatar>
            <img className={classes.icon} src={mapIcons[mapIconKey(item.category)]} alt="" />
          </Avatar>
        </ListItemAvatar>
        <ListItemText
          primary={devicePrimary}
          secondary={secondaryText()}
          slotProps={{
            primary: { noWrap: true },
            secondary: { noWrap: true },
          }}
        />
        {position && (
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'flex-end' }}>
            {/* Top row: Speed and Altitude */}
            <div style={{ display: 'flex', gap: '8px' }}>
              {position?.speed !== undefined && <PositionSpeed speed={position.speed} />}
              {position?.altitude !== undefined && <PositionAltitude altitude={position.altitude} />}
            </div>
            {/* Bottom row: Alarm, Ignition, Battery */}
            <div style={{ display: 'flex', gap: '8px', marginTop: '4px' }}>
              {position.attributes?.alarm && <PositionAlarm alarm={position.attributes.alarm} classes={classes} />}
              {position.attributes?.ignition !== undefined && (
                <PositionIgnition ignition={position.attributes.ignition} classes={classes} />
              )}
              {position.attributes?.batteryLevel !== undefined && (
                <PositionBattery
                  batteryLevel={position.attributes.batteryLevel}
                  charge={position.attributes.charge}
                  classes={classes}
                />
              )}
            </div>
          </div>
        )}
      </ListItemButton>
    </div>
  );
};

export default DeviceRow;
