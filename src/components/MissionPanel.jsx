import React, { useState, Fragment, useRef, useContext, useEffect } from 'react';
import RoutesList from './RoutesList';
import { Divider, Typography, IconButton, Toolbar, Switch } from '@mui/material';
import makeStyles from '@mui/styles/makeStyles';
import { useNavigate } from 'react-router-dom';
import { RosControl, RosContext } from './RosControl';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import UploadFileIcon from '@mui/icons-material/UploadFile';
import SaveAltIcon from '@mui/icons-material/SaveAlt';
import DeleteIcon from '@mui/icons-material/Delete';
import { MissionContext } from '../components/MissionController';

const useStyles = makeStyles((theme) => ({
  toolbar: {
    display: 'flex',
    gap: '10px 10px',
    height: '30px',
    borderBottom: '3px solid rgb(212, 212, 212)',
  },
  list: {
    maxHeight: `calc(100vh - 152px)`,
    overflowY: 'auto',
  },
  title: {
    flexGrow: 1,
  },
  fileInput: {
    display: 'none',
  },
}));

export const MissionPanel = ({ SetOpenSave }) => {
  const classes = useStyles();
  const navigate = useNavigate();
  const rosContex = useContext(RosContext);
  const scroolRef = useRef(null);
  const [checked, setChecked] = React.useState(false);
  const missionContext = useContext(MissionContext);

  const toggleChecked = () => {
    setChecked((prev) => !prev);
  };

  const [mission, setmission] = useState({
    name: 'no mission',
    description: '',
    route: [],
  });

  useEffect(() => {
    if (missionContext.changeWp.route_id >= 0) {
      if (checked) {
        let auxroute = JSON.parse(JSON.stringify(mission.route));
        // change all wapypoints
        let dif_lat =
          missionContext.changeWp.lat -
          auxroute[missionContext.changeWp.route_id]['wp'][missionContext.changeWp.wp_id]['pos'][0];
        let dif_lng =
          missionContext.changeWp.lng -
          auxroute[missionContext.changeWp.route_id]['wp'][missionContext.changeWp.wp_id]['pos'][1];
        let listWp = auxroute[missionContext.changeWp.route_id]['wp'].map((newWp) => {
          let uaxWp = newWp;
          uaxWp['pos'][0] = dif_lat + newWp['pos'][0];
          uaxWp['pos'][1] = dif_lng + newWp['pos'][1];
          return uaxWp;
        });
        auxroute[missionContext.changeWp.route_id]['wp'] = listWp;

        setmission({ ...mission, route: auxroute });
      } else {
        // change only this waypoint
        let auxroute = JSON.parse(JSON.stringify(mission.route));
        // change all wapypoints
        auxroute[missionContext.changeWp.route_id]['wp'][missionContext.changeWp.wp_id]['pos'][0] =
          missionContext.changeWp.lat;
        auxroute[missionContext.changeWp.route_id]['wp'][missionContext.changeWp.wp_id]['pos'][1] =
          missionContext.changeWp.lng;
        setmission({ ...mission, route: auxroute });
      }
    }
  }, [missionContext.changeWp]);

  const setScrool = (value) => {
    console.log('scrool' + value);
    if (scroolRef.current.scroll) {
      setTimeout(() => {
        scroolRef.current.scroll(0, value);
      }, 1000);
    }
  };

  const readFile = (e) => {
    //https://www.youtube.com/watch?v=K3SshoCXC2g
    const file = e.target.files[0];
    if (!file) return;

    const fileReader = new FileReader();
    fileReader.readAsText(file);
    fileReader.onload = () => {
      console.log(fileReader.result);
      console.log(file.name);
      rosContex.openMision(file.name, fileReader.result);
    };
    fileReader.onerror = () => {
      console.log(fileReader.error);
    };
  };

  const DeleteMission = () => {
    setmission({ name: 'no mission', description: '', route: [] });
  };
  const SaveMission = () => {
    console.log('save mission');
    SetOpenSave(true);
  };
  return (
    <Fragment>
      <Toolbar className={classes.toolbar}>
        <IconButton edge='start' sx={{ mr: 2 }} onClick={() => navigate(-1)}>
          <ArrowBackIcon />
        </IconButton>
        <Typography variant='h6' className={classes.title}>
          Mission Task
        </Typography>

        <Typography>Group Route</Typography>
        <Switch
          checked={checked}
          onChange={toggleChecked}
          name='checkedA'
          inputProps={{ 'aria-label': 'secondary checkbox' }}
        />
        <IconButton onClick={SaveMission}>
          <SaveAltIcon />
        </IconButton>
        <IconButton onClick={DeleteMission}>
          <DeleteIcon />
        </IconButton>
        <label htmlFor='upload-gpx'>
          <input
            accept='.yaml, .plan, .waypoint, .kml'
            id='upload-gpx'
            type='file'
            className={classes.fileInput}
            onChange={readFile}
          />
          <IconButton edge='end' component='span' onClick={() => {}}>
            <UploadFileIcon />
          </IconButton>
        </label>
      </Toolbar>
      <div ref={scroolRef} className={classes.list}>
        <RoutesList mission={mission} setmission={setmission} setScrool={setScrool} />
      </div>
    </Fragment>
  );
};
