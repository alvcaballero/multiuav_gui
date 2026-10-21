import { memo } from 'react';
import { useSelector } from 'react-redux';
import { green, red } from '@mui/material/colors';
import { Chip, IconButton, Tooltip } from '@mui/material';

import HomeIcon from '@mui/icons-material/Home';
import FolderOpenIcon from '@mui/icons-material/FolderOpen';
import DeleteForeverIcon from '@mui/icons-material/DeleteForever';
import CircleIcon from '@mui/icons-material/Circle';

// One connection of the GCS (WebSocket, ROS bridge, MQTT broker): green when up, red when down.
const ConnectionChip = ({ className, connected, label, tooltip }) => {
  const chip = (
    <Chip
      icon={
        <CircleIcon sx={{ fontSize: '7px !important', color: connected ? green[500] : red[500] }} />
      }
      label={label}
      size="small"
      className={className}
      sx={{
        backgroundColor: connected ? 'rgba(76,175,80,0.12)' : 'rgba(244,67,54,0.10)',
        color: connected ? green[800] : red[700],
        border: `1px solid ${connected ? 'rgba(76,175,80,0.35)' : 'rgba(244,67,54,0.35)'}`,
      }}
    />
  );
  return tooltip ? (
    <Tooltip title={tooltip} placement="bottom">
      {chip}
    </Tooltip>
  ) : (
    chip
  );
};

const MenuLeftGroup = ({
  classes,
  socketState,
  is3D,
  hasMission,
  onGoHome,
  onOpenFile,
  onClearMission,
}) => {
  const rosState = useSelector((state) => state.session.serverROS);
  // null = MQTT disabled on the server: no chip.
  const mqttState = useSelector((state) => state.session.serverMQTT);

  return (
    <div className={classes.leftGroup}>
      <ConnectionChip
        className={classes.rosChip}
        connected={socketState}
        label={socketState ? 'WS' : 'offline'}
        tooltip={socketState ? 'WebSocket connected' : 'WebSocket disconnected'}
      />

      <div className={classes.vDivider} />

      <ConnectionChip
        className={classes.rosChip}
        connected={rosState}
        label={rosState ? 'ROS' : 'offline'}
      />

      {mqttState !== null && (
        <>
          <div className={classes.vDivider} />
          <ConnectionChip
            className={classes.rosChip}
            connected={mqttState}
            label="MQTT"
            tooltip={mqttState ? 'MQTT broker connected' : 'MQTT broker disconnected'}
          />
        </>
      )}

      <div className={classes.vDivider} />

      <Tooltip title="Home position" placement="bottom">
        <span>
          <IconButton className={classes.iconBtn} size="small" onClick={onGoHome} disabled={is3D}>
            <HomeIcon sx={{ fontSize: 17 }} />
          </IconButton>
        </span>
      </Tooltip>

      <Tooltip title="Open mission file" placement="bottom">
        <IconButton
          aria-label="Open mission file"
          className={classes.iconBtn}
          size="small"
          component="label"
        >
          <FolderOpenIcon sx={{ fontSize: 17 }} />
          <input
            type="file"
            multiple={false}
            aria-label="Open mission file"
            style={{ display: 'none' }}
            onChange={onOpenFile}
          />
        </IconButton>
      </Tooltip>

      <Tooltip title="Clear mission" placement="bottom">
        <span>
          <IconButton
            className={classes.iconBtn}
            size="small"
            onClick={onClearMission}
            disabled={!hasMission}
            sx={{ '&:hover': { color: red[600], backgroundColor: red[50] } }}
          >
            <DeleteForeverIcon sx={{ fontSize: 17 }} />
          </IconButton>
        </span>
      </Tooltip>
    </div>
  );
};

export default memo(MenuLeftGroup);
