import React, { useState, useCallback } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import {
  Card,
  CardContent,
  Typography,
  CardActions,
  IconButton,
  Table,
  TableBody,
  TableRow,
  TableCell,
  Menu,
  MenuItem,
  Button,
  Chip,
  Tooltip,
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import CloseIcon from '@mui/icons-material/Close';
import ReplayIcon from '@mui/icons-material/Replay';
import PublishIcon from '@mui/icons-material/Publish';
import EditIcon from '@mui/icons-material/Edit';
import DeleteIcon from '@mui/icons-material/Delete';
import PendingIcon from '@mui/icons-material/Pending';
import GpsFixedIcon from '@mui/icons-material/GpsFixed';
import GpsNotFixedIcon from '@mui/icons-material/GpsNotFixed';
import SensorsOffIcon from '@mui/icons-material/SensorsOff';

import PositionValue from '../../common/components/PositionValue';
import RemoveDialog from '../../common/components/RemoveDialog';

import { devicesActions } from '../../store';
import CommandCard from '../commands/CommandCard';
import { CommandType } from '../../common/util/commandTypes';

const useStyles = makeStyles()((theme, { desktopPadding }) => ({
  card: {
    pointerEvents: 'auto',
    width: theme.dimensions.popupMaxWidth,
  },
  media: {
    height: theme.dimensions.popupImageHeight,
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: 'difference',
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: theme.spacing(1, 1, 0, 2),
  },
  content: {
    paddingTop: theme.spacing(1),
    paddingBottom: theme.spacing(1),
  },
  negative: {
    color: theme.palette.colors.negative,
  },
  icon: {
    width: '25px',
    height: '25px',
    filter: 'brightness(0) invert(1)',
  },
  table: {
    '& .MuiTableCell-sizeSmall': {
      paddingLeft: 0,
      paddingRight: 0,
    },
  },
  cell: {
    borderBottom: 'none',
  },
  actions: {
    justifyContent: 'space-between',
  },
  root: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 5,
    left: '50%',
    [theme.breakpoints.up('md')]: {
      left: `calc(50% + ${desktopPadding} / 2)`,
      bottom: theme.spacing(3),
    },
    [theme.breakpoints.down('md')]: {
      left: '50%',
      bottom: `calc(${theme.spacing(3)} + ${theme.dimensions.bottomBarHeight}px)`,
    },
    transform: 'translateX(-50%)',
  },
}));

const StatusRow = ({ name, content }) => {
  const { classes } = useStyles({ desktopPadding: 0 });
  return (
    <TableRow>
      <TableCell className={classes.cell}>
        <Typography variant="body2">{name}</Typography>
      </TableCell>
      <TableCell className={classes.cell}>
        <Typography variant="body2" color="textSecondary">
          {content}
        </Typography>
      </TableCell>
    </TableRow>
  );
};

const MemoCardActions = React.memo(
  ({ onOpenMenu, onSyncFiles, onOpenCommand, onEdit, onRemove, disableActions, nullposition }) => (
    <CardActions disableSpacing sx={{ justifyContent: 'space-between' }}>
      <IconButton color="secondary" onClick={onOpenMenu} disabled={nullposition}>
        <PendingIcon />
      </IconButton>
      <IconButton onClick={onSyncFiles}>
        <ReplayIcon />
      </IconButton>
      <IconButton onClick={onOpenCommand} disabled={disableActions}>
        <PublishIcon />
      </IconButton>
      <IconButton onClick={onEdit}>
        <EditIcon />
      </IconButton>
      <IconButton color="error" onClick={onRemove}>
        <DeleteIcon />
      </IconButton>
    </CardActions>
  ),
);
const MemoCardHeader = React.memo(
  ({ classes, deviceName, onClose, changeMapFollow, mapFollow }) => {
    return (
      <div className={classes.header}>
        <Typography variant="body2" color="textSecondary">
          {deviceName}
        </Typography>
        <IconButton size="small" onClick={changeMapFollow}>
          {mapFollow ? <GpsFixedIcon fontSize="small" /> : <GpsNotFixedIcon fontSize="small" />}
        </IconButton>
        <IconButton size="small" onClick={onClose} onTouchStart={onClose}>
          <CloseIcon fontSize="small" />
        </IconButton>
      </div>
    );
  },
);

// Slots of the category that stopped (`stale`) or never started (`missing`) arriving,
// as the server's telemetry watchdog reports them in `device.telemetry`.
const TELEMETRY_HEALTH_TEXT = { stale: 'stopped', missing: 'never arrived' };

const TelemetryChip = ({ telemetry }) => {
  const slots = Object.keys(telemetry ?? {});
  if (slots.length === 0) return null;
  return (
    <Tooltip
      title={slots
        .map((slot) => `${slot}: ${TELEMETRY_HEALTH_TEXT[telemetry[slot]] ?? telemetry[slot]}`)
        .join(' · ')}
    >
      <Chip
        icon={<SensorsOffIcon />}
        color="warning"
        size="small"
        variant="outlined"
        label={`No data: ${slots.join(', ')}`}
        sx={{ mx: 2, mt: 1, maxWidth: 'calc(100% - 32px)' }}
      />
    </Tooltip>
  );
};

const serverCommand = async (deviceId, command, attributes) => {
  console.log('send command uavud: ' + deviceId + command);
  try {
    const response = await fetch('/api/commands/send', {
      method: 'POST',
      body: JSON.stringify({ deviceId, type: command, attributes }),
      headers: {
        'Content-Type': 'application/json',
      },
    });
    if (response.ok) {
      const myResponse = await response.json();
      console.log(myResponse);
    } else {
      console.log('Error1:' + response);
      throw Error(await response.text());
    }
  } catch (error) {
    console.log('Error2:' + error);
  }
};

const StatusCard = ({ deviceId, position, onClose, desktopPadding = 0, is3d = false }) => {
  const { classes } = useStyles({ desktopPadding });
  const navigate = useNavigate();
  const dispatch = useDispatch();

  const device = useSelector((state) => state.devices.items[deviceId]);
  const mapFollowstats = useSelector((state) => state.devices.follow);

  const positionItems = 'speed,course,batteryLevel,gimbal,MIC_1,MIC_2,MIC_3,Metano,Alcohol,CO';

  const [anchorEl, setAnchorEl] = useState(null);
  const [removing, setRemoving] = useState(false);
  const [openSendCommand, setOpenSendCommand] = useState(false);
  const nullPosition = !position;
  const disableActions = false;
  const mapFollow = mapFollowstats;
  const deviceName = device.name;

  const handleOpenMenu = (e) => setAnchorEl(e.currentTarget);
  const handleSyncFiles = useCallback(
    () => serverCommand(deviceId, CommandType.SYNC_FILES),
    [deviceId],
  );
  const handleRemoving = () => setRemoving(true);
  const changeMapFollow = useCallback(
    () => dispatch(devicesActions.updateFollow(!mapFollow)),
    [mapFollow, dispatch],
  );

  const openCommand = () => setOpenSendCommand(true);
  const navigateToDevice = useCallback(() => {
    if (is3d) {
      navigate(`/device3d/${deviceId}`);
    } else {
      navigate(`/device/${deviceId}`);
    }
  }, [deviceId, navigate, is3d]);

  const handleRemove = async (removed) => {
    if (removed) {
      const response = await fetch('/api/devices');
      if (response.ok) {
        let myresponse = await response.json();
        dispatch(devicesActions.refresh(Object.values(myresponse)));
        onClose();
      } else {
        throw Error(await response.text());
      }
    }
    setRemoving(false);
  };

  return (
    <>
      <div className={classes.root}>
        {device && (
          <Card elevation={3} className={classes.card}>
            <MemoCardHeader
              classes={classes}
              deviceName={deviceName}
              onClose={onClose}
              changeMapFollow={changeMapFollow}
              mapFollow={mapFollow}
            />
            <TelemetryChip telemetry={device.telemetry} />
            {position && (
              <CardContent className={classes.content}>
                <Table size="small" classes={{ root: classes.table }}>
                  <TableBody>
                    {position.hasOwnProperty('latitude') && (
                      <StatusRow
                        key="latitude"
                        name={'Position'}
                        content={
                          <a
                            href={
                              'https://www.google.com/maps?q=' +
                              position.latitude +
                              ',' +
                              position.longitude
                            }
                            target="_blank"
                          >
                            {'[' +
                              position.latitude.toFixed(6) +
                              ',' +
                              position.longitude.toFixed(6) +
                              ']'}
                          </a>
                        }
                      />
                    )}

                    {positionItems
                      .split(',')
                      .filter(
                        (key) =>
                          position.hasOwnProperty(key) || position.attributes.hasOwnProperty(key),
                      )
                      .map((key) => (
                        <StatusRow
                          key={key}
                          name={key}
                          content={
                            <PositionValue
                              position={position}
                              property={position.hasOwnProperty(key) ? key : null}
                              attribute={position.hasOwnProperty(key) ? null : key}
                            />
                          }
                        />
                      ))}

                    {position.attributes.hasOwnProperty('alarm') &&
                      device.category.includes('catec') && (
                        <StatusRow
                          key="alarm1"
                          name={'Alarm ' + position.attributes.alarm}
                          content={
                            <div style={{ width: '100%' }}>
                              <Button
                                variant="contained"
                                size="small"
                                color="primary"
                                style={{ margin: '1px', display: 'inline-block' }}
                                onClick={() =>
                                  serverCommand(device.id, CommandType.THREAT_CONFIRMATION)
                                }
                              >
                                Validate
                              </Button>
                              <Button
                                variant="contained"
                                size="small"
                                color="secondary"
                                style={{ margin: '1px', display: 'inline-block' }}
                                onClick={() => serverCommand(device.id, CommandType.THREAT_DEFUSE)}
                              >
                                Dismiss
                              </Button>
                            </div>
                          }
                        />
                      )}
                  </TableBody>
                </Table>
              </CardContent>
            )}
            <MemoCardActions
              onOpenMenu={handleOpenMenu}
              onSyncFiles={handleSyncFiles}
              onOpenCommand={openCommand}
              onEdit={navigateToDevice}
              onRemove={handleRemoving}
              disableActions={disableActions}
              nullposition={nullPosition}
            />
          </Card>
        )}
      </div>
      {position && anchorEl && (
        <Menu anchorEl={anchorEl} open={Boolean(anchorEl)} onClose={() => setAnchorEl(null)}>
          <MenuItem onClick={navigateToDevice}>
            <Typography color="secondary">Detalle Dispositivo</Typography>
          </MenuItem>
          <MenuItem
            component="a"
            target="_blank"
            href={`https://www.google.com/maps/search/?api=1&query=${position.latitude}%2C${position.longitude}`}
          >
            Google Maps
          </MenuItem>
          <MenuItem
            component="a"
            target="_blank"
            href={`http://maps.apple.com/?ll=${position.latitude},${position.longitude}`}
          >
            Apple Maps
          </MenuItem>
          <MenuItem
            component="a"
            target="_blank"
            href={`https://www.google.com/maps/@?api=1&map_action=pano&viewpoint=${position.latitude}%2C${position.longitude}&heading=${position.course}`}
          >
            Street View
          </MenuItem>
        </Menu>
      )}
      {openSendCommand && <CommandCard id={deviceId} onClose={() => setOpenSendCommand(false)} />}

      <RemoveDialog
        open={removing}
        endpoint="devices"
        itemId={deviceId}
        onResult={(removed) => handleRemove(removed)}
      />
    </>
  );
};

export default StatusCard;
