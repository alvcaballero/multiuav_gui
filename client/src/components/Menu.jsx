import { Height } from '@mui/icons-material';
import React, { useEffect, useState, useContext } from 'react';
import { map } from '../Mapview/MapView';
import { RosContext } from './RosControl';
import { useSelector } from 'react-redux';
import HomeIcon from '@mui/icons-material/Home';
import FolderIcon from '@mui/icons-material/Folder';
import FlightIcon from '@mui/icons-material/Flight';
import FileUploadIcon from '@mui/icons-material/FileUpload';
import TabIcon from '@mui/icons-material/Tab';
import ReplyIcon from '@mui/icons-material/Reply';
import CallMadeIcon from '@mui/icons-material/CallMade';
import ModeEditIcon from '@mui/icons-material/ModeEdit';
import CachedIcon from '@mui/icons-material/Cached';
import { grey } from '@mui/material/colors';
import makeStyles from '@mui/styles/makeStyles';
import { usePreference } from '../common/preferences';

import { Card, IconButton, Button, ButtonGroup, CardMedia } from '@mui/material';

const useStyles = makeStyles((theme) => ({
  toolbar: {
    height: '36px',
    //backgroundColor: grey[300],
    background: 'linear-gradient(#E7E5E7, #D3D1D3)',
    borderTop: '1px solid rgba(255, 255, 255, .6)',
  },
  mediaButton: {
    width: 'auto',
    color: grey[700],
    backgroundColor: '#F9F9F9', //grey[50],
    boxShadow: 'inset 0px -4px 4px 0 rgba(0, 0, 0, 0.1)',
    maxHeight: '22px',
    top: '2px',
    border: '1px solid #C2C0C2',
    margin: '5px 10px',
    fontSize: '12px',
    '&:hover': {
      backgroundColor: grey[300],
    },
  },
  mediaicon: {
    color: grey[700],
    width: '15px',
  },
}));

export const Menu = ({ SetAddUAVOpen }) => {
  const classes = useStyles();
  const rosContex = useContext(RosContext);
  const [MissionName, setMissionName] = useState('no load mission');
  //const [MissionHome, setMissionHome] = useState([0,0]);
  const Mission_Name = useSelector((state) => state.mission.name);
  const Mission_Home = useSelector((state) => state.mission.home);
  const [hidestatus, sethidestatus] = useState(true);

  const defaultLatitude = usePreference('latitude',0);
  const defaultLongitude = usePreference('longitude',0);
  const defaultZoom = usePreference('zoom', 10);

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

  function HomeMap() {
    map.easeTo({
      center: [defaultLongitude, defaultLatitude],
      zoom: Math.max(map.getZoom(), defaultZoom),
      offset: [0, -1 / 2],
    });
  }

  function MissionMap() {
    map.easeTo({
      center: [Mission_Home[1], Mission_Home[0]],
      zoom: Math.max(map.getZoom(), defaultZoom),
      offset: [0, -1 / 2],
    });
  }

  function openAddUav() {
    SetAddUAVOpen(true);
  }

  const hideStatusWindow = () => {
    sethidestatus(!hidestatus);
  };

  useEffect(() => {
    setMissionName(Mission_Name);
  }, [Mission_Name]);
  useEffect(() => {
    MissionMap();
  }, [Mission_Home]);

  return (
    <header className={classes.toolbar}>
      <div>
        <ButtonGroup
          style={{
            height: '26px',
            display: 'inline',
            justifyContent: 'flex-end',
          }}
        >
          <Button onClick={HomeMap} className={classes.mediaButton} style={{ margin: '0px 0px' }}>
            <HomeIcon fontSize="small" className={classes.mediaicon} />
          </Button>
          <Button id="openMission" className={classes.mediaButton} style={{ margin: '0px 0px' }}>
            <label htmlFor="openMissionNavbar" style={{ padding: 0 }}>
              <FolderIcon fontSize="small" className={classes.mediaicon} style={{ position: 'relative', top: '3px' }} />
            </label>
            <input
              type="file"
              multiple={false}
              style={{ display: 'none' }}
              id="openMissionNavbar"
              onChange={readFile}
            />
          </Button>
          <Button id="openAddUav" onClick={openAddUav} className={classes.mediaButton} style={{ margin: '0px 0px' }}>
            <FlightIcon fontSize="small" className={classes.mediaicon} style={{ transform: 'rotate(90deg)' }} />
          </Button>
        </ButtonGroup>

        <RosContext.Consumer>
          {({ rosState }) => (
            <Button id="rosConnect" onClick={rosContex.rosConnect} className={classes.mediaButton}>
              {rosState && 'conectado'} {!rosState && 'desconectado'}{' '}
            </Button>
          )}
        </RosContext.Consumer>

        <Button id="loadMission" size="small" onClick={openAddUav} className={classes.mediaButton}>
          <FileUploadIcon fontSize="small" className={classes.mediaicon} />
        </Button>

        <Button id="commandMission" onClick={openAddUav} className={classes.mediaButton}>
          Fly!
        </Button>

        <Button
          id="openedmission"
          onClick={MissionMap}
          className={classes.mediaButton}
          style={{ margin: '0px 100px', width: '500px' }}
        >
          {' '}
          {MissionName}
        </Button>
        <Button variant="filledTonal" onClick={(e) => hideStatusWindow()} className={classes.mediaButton}>
          Status
        </Button>

        <Button id="openTerminal" style={{ visibility: true }} className={classes.mediaButton}>
          <TabIcon fontSize="small" className={classes.mediaicon} />
        </Button>
        <Button id="UndoMission" style={{ visibility: true }} className={classes.mediaButton}>
          <ReplyIcon fontSize="small" className={classes.mediaicon} />
        </Button>

        <Button id="reset" className={classes.mediaButton}>
          <CallMadeIcon fontSize="small" className={classes.mediaicon} />
        </Button>

        <Button id="showManualMode" className={classes.mediaButton}>
          <ModeEditIcon fontSize="small" className={classes.mediaicon} />
        </Button>

        <Button id="commandManualMission" style={{ visibility: true }} className={classes.mediaButton}>
          <CachedIcon fontSize="small" className={classes.mediaicon} />
        </Button>
      </div>
    </header>
  );
};
