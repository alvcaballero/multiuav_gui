import { useSelector } from 'react-redux';
import { green, red } from '@mui/material/colors';
import { Chip, IconButton, Tooltip } from '@mui/material';

import HomeIcon from '@mui/icons-material/Home';
import FolderOpenIcon from '@mui/icons-material/FolderOpen';
import DeleteForeverIcon from '@mui/icons-material/DeleteForever';
import CircleIcon from '@mui/icons-material/Circle';

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

  return (
    <div className={classes.leftGroup}>
      <Tooltip
        title={socketState ? 'WebSocket connected' : 'WebSocket disconnected'}
        placement="bottom"
      >
        <Chip
          icon={
            <CircleIcon
              sx={{ fontSize: '7px !important', color: socketState ? green[500] : red[500] }}
            />
          }
          label={socketState ? 'WS' : 'offline'}
          size="small"
          className={classes.rosChip}
          sx={{
            backgroundColor: socketState ? 'rgba(76,175,80,0.12)' : 'rgba(244,67,54,0.10)',
            color: socketState ? green[800] : red[700],
            border: `1px solid ${socketState ? 'rgba(76,175,80,0.35)' : 'rgba(244,67,54,0.35)'}`,
          }}
        />
      </Tooltip>

      <div className={classes.vDivider} />

      <Chip
        icon={
          <CircleIcon
            sx={{ fontSize: '7px !important', color: rosState ? green[500] : red[500] }}
          />
        }
        label={rosState ? 'ROS' : 'offline'}
        size="small"
        className={classes.rosChip}
        sx={{
          backgroundColor: rosState ? 'rgba(76,175,80,0.12)' : 'rgba(244,67,54,0.10)',
          color: rosState ? green[800] : red[700],
          border: `1px solid ${rosState ? 'rgba(76,175,80,0.35)' : 'rgba(244,67,54,0.35)'}`,
        }}
      />

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

export default MenuLeftGroup;
