import React from 'react';
import { Box, Button, Typography } from '@mui/material';
import ReplayIcon from '@mui/icons-material/Replay';
import { makeStyles } from 'tss-react/mui';
import BaseSettings from './BaseSettings';

const useStyles = makeStyles()((theme) => ({
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
  tabPanelContent: {
    maxHeight: 'calc(100vh - 240px)',
    overflowY: 'auto',
    paddingRight: '8px',
  },
  panelButton: {
    width: '80%',
    flexShrink: 0,
    marginTop: '15px',
  },
}));

const SettingsTab = ({
  missionRequest,
  markers,
  notification,
  onSetBaseSettings,
  onGoToBase,
  onSendPlanning,
  onResetPolling,
  onRequestMission,
  onSaveGlobalMarkers,
}) => {
  const { classes } = useStyles();

  return (
    <div className={`${classes.details} ${classes.tabPanelContent}`}>
      <BaseSettings
        data={missionRequest.assignments || []}
        markers={markers}
        param={missionRequest.settingsSchema}
        defaultSettings={missionRequest.defaultSettings}
        setData={onSetBaseSettings}
        goToBase={onGoToBase}
      />

      <Box sx={{ textAlign: 'center' }}>
        <div className={classes.panelButton} style={{ marginLeft: 'auto', marginRight: 'auto' }}>
          <Button variant="contained" sx={{ width: '80%' }} onClick={onSendPlanning}>
            Planning
          </Button>
          <Button
            variant="contained"
            color="secondary"
            size="large"
            sx={{ width: '20%' }}
            endIcon={<ReplayIcon />}
            onClick={onResetPolling}
          />
        </div>

        <Button
          variant="contained"
          size="large"
          className={classes.panelButton}
          onClick={onRequestMission}
        >
          Planning with Global Setting
        </Button>
        <Button
          variant="contained"
          size="large"
          className={classes.panelButton}
          onClick={onSaveGlobalMarkers}
        >
          Save Global Settings
        </Button>
      </Box>
      <Typography>{notification}</Typography>
    </div>
  );
};

export default SettingsTab;
