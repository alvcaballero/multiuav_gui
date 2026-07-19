import { memo } from 'react';
import { Box, Button, IconButton, Tooltip } from '@mui/material';
import { red } from '@mui/material/colors';

import FileUploadIcon from '@mui/icons-material/FileUpload';
import SendIcon from '@mui/icons-material/Send';
import PauseIcon from '@mui/icons-material/Pause';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import StopIcon from '@mui/icons-material/Stop';
import RouteIcon from '@mui/icons-material/Route';
import ModeEditIcon from '@mui/icons-material/ModeEdit';
import MapIcon from '@mui/icons-material/Map';
import ViewInArIcon from '@mui/icons-material/ViewInAr';
import AssignmentIcon from '@mui/icons-material/Assignment';
import NotificationsIcon from '@mui/icons-material/Notifications';

const MenuRightGroup = ({
  classes,
  is3D,
  hasMission,
  loadingMission,
  isRunning,
  unseenErrorsCount,
  unseenMissionsCount,
  onLoadMission,
  onRequestFly,
  onPauseMission,
  onResumeMission,
  onStopMission,
  onNavigatePlanning,
  onNavigateMission,
  onToggleView,
  onOpenActiveMissions,
  onOpenEvents,
}) => (
  <div className={classes.rightGroup}>
    <Tooltip title="Load mission to UAVs" placement="bottom">
      <span>
        <Button
          className={classes.loadBtn}
          startIcon={<FileUploadIcon sx={{ fontSize: 14 }} />}
          onClick={onLoadMission}
          disabled={!hasMission || loadingMission}
          variant="outlined"
        >
          {loadingMission ? 'Loading…' : 'Load'}
        </Button>
      </span>
    </Tooltip>

    <Tooltip title="Command mission — Fly!" placement="bottom">
      <span>
        <Button
          className={classes.flyBtn}
          startIcon={<SendIcon sx={{ fontSize: 13 }} />}
          onClick={onRequestFly}
          disabled={!hasMission}
        >
          Fly!
        </Button>
      </span>
    </Tooltip>

    <div className={classes.vDivider} />

    <Tooltip title="Pause mission" placement="bottom">
      <span>
        <IconButton
          className={classes.pauseBtn}
          size="small"
          onClick={onPauseMission}
          disabled={!isRunning}
        >
          <PauseIcon sx={{ fontSize: 17 }} />
        </IconButton>
      </span>
    </Tooltip>

    <Tooltip title="Resume mission" placement="bottom">
      <span>
        <IconButton
          className={classes.resumeBtn}
          size="small"
          onClick={onResumeMission}
          disabled={!isRunning}
        >
          <PlayArrowIcon sx={{ fontSize: 17 }} />
        </IconButton>
      </span>
    </Tooltip>

    <Tooltip title="Stop mission" placement="bottom">
      <span>
        <IconButton
          className={classes.stopBtn}
          size="small"
          onClick={onStopMission}
          disabled={!isRunning}
        >
          <StopIcon sx={{ fontSize: 17 }} />
        </IconButton>
      </span>
    </Tooltip>

    <div className={classes.vDivider} />

    <Tooltip title="Planning" placement="bottom">
      <IconButton className={classes.iconBtn} size="small" onClick={onNavigatePlanning}>
        <RouteIcon sx={{ fontSize: 17 }} />
      </IconButton>
    </Tooltip>

    <Tooltip title="Edit mission" placement="bottom">
      <IconButton className={classes.iconBtn} size="small" onClick={onNavigateMission}>
        <ModeEditIcon sx={{ fontSize: 17 }} />
      </IconButton>
    </Tooltip>

    <Tooltip title={is3D ? 'Switch to 2D map' : 'Switch to 3D view'} placement="bottom">
      <Button
        className={classes.toggleBtn}
        startIcon={
          is3D ? <MapIcon sx={{ fontSize: 13 }} /> : <ViewInArIcon sx={{ fontSize: 13 }} />
        }
        onClick={onToggleView}
      >
        {is3D ? '2D' : '3D'}
      </Button>
    </Tooltip>

    <div className={classes.vDivider} />

    <Tooltip
      title={unseenMissionsCount ? `${unseenMissionsCount} new mission(s)` : 'Active missions'}
      placement="bottom"
    >
      <IconButton
        className={classes.iconBtn}
        size="small"
        onClick={onOpenActiveMissions}
        sx={{ position: 'relative' }}
      >
        <AssignmentIcon sx={{ fontSize: 17 }} />
        {unseenMissionsCount > 0 && (
          <Box
            sx={{
              position: 'absolute',
              top: 3,
              right: 3,
              width: 7,
              height: 7,
              borderRadius: '50%',
              backgroundColor: red[500],
              border: '1.5px solid #e4e7ec',
            }}
          />
        )}
      </IconButton>
    </Tooltip>

    <Tooltip
      title={unseenErrorsCount ? `${unseenErrorsCount} new error(s)` : 'Events log'}
      placement="bottom"
    >
      <IconButton
        className={classes.iconBtn}
        size="small"
        onClick={onOpenEvents}
        sx={{ position: 'relative' }}
      >
        <NotificationsIcon sx={{ fontSize: 17 }} />
        {unseenErrorsCount > 0 && (
          <Box
            sx={{
              position: 'absolute',
              top: 3,
              right: 3,
              width: 7,
              height: 7,
              borderRadius: '50%',
              backgroundColor: red[500],
              border: '1.5px solid #e4e7ec',
            }}
          />
        )}
      </IconButton>
    </Tooltip>
  </div>
);

export default memo(MenuRightGroup);
