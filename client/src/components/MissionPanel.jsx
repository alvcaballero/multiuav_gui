import { Fragment, useRef, useCallback } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import RoutesList from './RoutesList';
import { Typography, IconButton, Toolbar, Switch } from '@mui/material';
import { makeStyles } from 'tss-react/mui';

import { useNavigate } from 'react-router-dom';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import UploadFileIcon from '@mui/icons-material/UploadFile';
import SaveAltIcon from '@mui/icons-material/SaveAlt';
import DeleteIcon from '@mui/icons-material/Delete';
import { FiletoMission } from '../Mapview/MissionConvert';
import { missionActions } from '../store';

const useStyles = makeStyles()((theme) => ({
  toolbar: {
    display: 'flex',
    gap: '10px 10px',
    height: '30px',
    borderBottom: '3px solid rgb(212, 212, 212)',
  },
  list: {
    maxHeight: 'calc(100vh - 152px)',
    overflowY: 'auto',
  },
  title: {
    flexGrow: 1,
  },
  fileInput: {
    display: 'none',
  },
}));

const MissionPanel = ({ SetOpenSave }) => {
  const { classes } = useStyles();
  const navigate = useNavigate();
  const scroolRef = useRef(null);
  const dispatch = useDispatch();

  // Read group route mode from Redux
  const groupRouteMode = useSelector((state) => state.mission.groupRouteMode);

  const toggleGroupRouteMode = () => {
    dispatch(missionActions.setGroupRouteMode(!groupRouteMode));
  };

  const setScrool = useCallback((value) => {
    if (scroolRef.current?.scroll) {
      setTimeout(() => {
        scroolRef.current.scroll(0, value);
      }, 1000);
    }
  }, []);

  const readFile = (e) => {
    const file = e.target.files[0];
    if (!file) return;

    const fileReader = new FileReader();
    fileReader.readAsText(file);
    fileReader.onload = () => {
      FiletoMission({ name: file.name, data: fileReader.result });
    };
    fileReader.onerror = () => {
      console.error(fileReader.error);
    };
  };

  const handleDeleteMission = () => {
    dispatch(missionActions.clearMission());
  };

  const handleSaveMission = () => {
    SetOpenSave(true);
  };

  return (
    <Fragment>
      <Toolbar className={classes.toolbar}>
        <IconButton edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
          <ArrowBackIcon />
        </IconButton>
        <Typography variant="h6" className={classes.title}>
          Mission Task
        </Typography>

        <Typography>Group Route</Typography>
        <Switch
          checked={groupRouteMode}
          onChange={toggleGroupRouteMode}
          name="groupRouteMode"
          inputProps={{ 'aria-label': 'group route mode' }}
        />
        <IconButton onClick={handleSaveMission}>
          <SaveAltIcon />
        </IconButton>
        <IconButton onClick={handleDeleteMission}>
          <DeleteIcon />
        </IconButton>
        <label htmlFor="upload-gpx">
          <input
            accept=".yaml, .plan, .waypoint, .kml"
            id="upload-gpx"
            type="file"
            className={classes.fileInput}
            onChange={readFile}
          />
          <IconButton edge="end" component="span">
            <UploadFileIcon />
          </IconButton>
        </label>
      </Toolbar>
      <div ref={scroolRef} className={classes.list}>
        <RoutesList setScrool={setScrool} />
      </div>
    </Fragment>
  );
};

export default MissionPanel;
