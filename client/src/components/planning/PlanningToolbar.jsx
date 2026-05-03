import React from 'react';
import { IconButton, Toolbar, Typography, Switch } from '@mui/material';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import SaveAltIcon from '@mui/icons-material/SaveAlt';
import DeleteIcon from '@mui/icons-material/Delete';
import { makeStyles } from 'tss-react/mui';
import UploadButtons from '../ui/uploadButton';

const useStyles = makeStyles()(() => ({
  toolbar: {
    display: 'flex',
    gap: '10px 10px',
    height: '30px',
    borderBottom: '3px solid rgb(212, 212, 212)',
  },
  title: {
    flexGrow: 1,
  },
}));

const PlanningToolbar = ({ onBack, onSave, onDelete, onReadFile, showMission, onToggleShowMission }) => {
  const { classes } = useStyles();

  return (
    <Toolbar className={classes.toolbar}>
      <IconButton edge="start" sx={{ mr: 2 }} onClick={onBack}>
        <ArrowBackIcon />
      </IconButton>
      <Typography variant="h6" className={classes.title}>
        Planning
      </Typography>
      <Typography>Show Mission</Typography>
      <Switch
        checked={showMission}
        onChange={onToggleShowMission}
        name="checkedA"
        slotProps={{ input: { 'aria-label': 'secondary checkbox' } }}
      />
      <IconButton onClick={onSave}>
        <SaveAltIcon />
      </IconButton>
      <IconButton onClick={onDelete}>
        <DeleteIcon />
      </IconButton>
      <UploadButtons readFile={onReadFile} typefiles=".yaml, .plan, .waypoint, .kml" />
    </Toolbar>
  );
};

export default PlanningToolbar;
