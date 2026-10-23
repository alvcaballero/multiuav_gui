import { memo } from 'react';
import { useSelector } from 'react-redux';
import { alpha } from '@mui/material/styles';
import { Chip, IconButton, Tooltip } from '@mui/material';

import HomeIcon from '@mui/icons-material/Home';
import FolderOpenIcon from '@mui/icons-material/FolderOpen';
import DeleteForeverIcon from '@mui/icons-material/DeleteForever';
import CircleIcon from '@mui/icons-material/Circle';
import { STATUS } from '../../../common/theme/palette';

// One connection of the GCS (WebSocket, ROS bridge, MQTT broker): green (STATUS.done) when up,
// red (STATUS.failed) when down — a tint of it behind, the full colour for the dot and the text.
const ConnectionChip = ({ className, connected, label, tooltip }) => {
  const tone = connected ? STATUS.done : STATUS.failed;
  const chip = (
    <Chip
      icon={<CircleIcon sx={{ fontSize: '7px !important', color: tone }} />}
      label={label}
      size="small"
      className={className}
      sx={{
        backgroundColor: alpha(tone, 0.1),
        color: tone,
        border: `1px solid ${alpha(tone, 0.35)}`,
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
            sx={{
              '&:hover': {
                color: 'error.main',
                backgroundColor: (theme) => alpha(theme.palette.error.main, 0.08),
              },
            }}
          >
            <DeleteForeverIcon sx={{ fontSize: 17 }} />
          </IconButton>
        </span>
      </Tooltip>
    </div>
  );
};

export default memo(MenuLeftGroup);
