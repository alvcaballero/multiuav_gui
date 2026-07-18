import { memo } from 'react';
import { grey } from '@mui/material/colors';
import { Box, Tooltip, Typography } from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import MyLocationIcon from '@mui/icons-material/MyLocation';
import CircleIcon from '@mui/icons-material/Circle';

import { missionStyle } from '../../../shared/missionStatus';

const MenuCenterStatus = ({
  classes,
  hasMission,
  is3D,
  missionName,
  currentStatus,
  onGoToMission,
  onOpenMissionDetail,
}) => (
  <div className={classes.center}>
    <Box
      sx={{
        display: 'flex',
        alignItems: 'stretch',
        height: '26px',
        border: `1px solid ${grey[300]}`,
        borderRadius: '6px',
        overflow: 'hidden',
        backgroundColor: '#fff',
        boxShadow: '0 1px 2px rgba(0,0,0,0.06)',
      }}
    >
      {/* Segmento localizar */}
      <Tooltip title={hasMission && !is3D ? 'Center map on mission' : ''} placement="bottom">
        <span style={{ display: 'flex' }}>
          <Box
            component="button"
            onClick={hasMission && !is3D ? onGoToMission : undefined}
            disabled={!hasMission || is3D}
            sx={{
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              width: '28px',
              border: 'none',
              borderRight: `1px solid ${grey[200]}`,
              backgroundColor: 'transparent',
              padding: 0,
              cursor: hasMission && !is3D ? 'pointer' : 'default',
              color: hasMission && !is3D ? grey[600] : grey[400],
              '&:hover':
                hasMission && !is3D ? { backgroundColor: grey[100], color: grey[900] } : {},
            }}
          >
            <MyLocationIcon sx={{ fontSize: 14 }} />
          </Box>
        </span>
      </Tooltip>

      {/* Segmento nombre */}
      <Tooltip title={hasMission ? 'Mission details' : ''} placement="bottom">
        <span style={{ display: 'flex' }}>
          <Box
            component="button"
            onClick={hasMission ? onOpenMissionDetail : undefined}
            disabled={!hasMission}
            sx={{
              display: 'flex',
              alignItems: 'center',
              gap: '5px',
              border: 'none',
              borderRight: `1px solid ${grey[200]}`,
              backgroundColor: 'transparent',
              padding: '0 10px',
              minWidth: '160px',
              maxWidth: '300px',
              cursor: hasMission ? 'pointer' : 'default',
              '&:hover': hasMission ? { backgroundColor: grey[50] } : {},
            }}
          >
            <Typography
              sx={{
                fontSize: '12px',
                fontWeight: 600,
                overflow: 'hidden',
                textOverflow: 'ellipsis',
                whiteSpace: 'nowrap',
                color: hasMission ? grey[800] : grey[400],
                flex: 1,
                textAlign: 'left',
              }}
            >
              {hasMission ? missionName : 'No mission loaded'}
            </Typography>
            {hasMission && (
              <ExpandMoreIcon sx={{ fontSize: 14, color: grey[400], flexShrink: 0 }} />
            )}
          </Box>
        </span>
      </Tooltip>

      {/* Segmento status */}
      <Box
        sx={{
          display: 'flex',
          alignItems: 'center',
          gap: '5px',
          padding: '0 9px',
          minWidth: '72px',
          justifyContent: 'center',
          backgroundColor: currentStatus ? missionStyle(currentStatus).color : grey[100],
          cursor: 'default',
        }}
      >
        {currentStatus && (
          <CircleIcon
            sx={{
              fontSize: 7,
              color: '#fff',
              flexShrink: 0,
              ...(currentStatus === 'running' && {
                animation: 'menuPulse 1.4s ease-in-out infinite',
                '@keyframes menuPulse': { '0%,100%': { opacity: 1 }, '50%': { opacity: 0.3 } },
              }),
            }}
          />
        )}
        <Typography
          sx={{
            fontSize: '10px',
            fontWeight: 700,
            letterSpacing: '0.4px',
            textTransform: 'uppercase',
            whiteSpace: 'nowrap',
            color: currentStatus ? '#fff' : grey[500],
          }}
        >
          {currentStatus ? missionStyle(currentStatus).label : 'no status'}
        </Typography>
      </Box>
    </Box>
  </div>
);

export default memo(MenuCenterStatus);
