import { Height } from '@mui/icons-material';
import React ,{ useEffect,useState ,useContext }from 'react'
import { map } from '../Mapview/Mapview.js'
import { RosContext } from './RosControl'
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

import {
  Card,
  IconButton,Button,ButtonGroup,
  CardMedia,
} from '@mui/material';


export const Menu = ({SetAddUAVOpen}) => {
  const rosContex = useContext(RosContext);
  const [MissionName, setMissionName] = useState('no load mission');
  const [MissionHome, setMissionHome] = useState([0,0]);
  const Mission_Name = useSelector((state) => state.mission.name);
  const Mission_Home = useSelector((state) => state.mission.home);
  const [hidestatus,sethidestatus]  = useState(true);

  const readFile = ( e ) => {//https://www.youtube.com/watch?v=K3SshoCXC2g
    const file = e.target.files[0];
    if ( !file ) return;
    const fileReader = new FileReader();
    fileReader.readAsText( file );
    fileReader.onload = () => {
      console.log( fileReader.result );
      console.log( file.name );
      rosContex.openMision(file.name,fileReader.result)
    }
    fileReader.onerror = () => {
      console.log( fileReader.error );
    }
  }

  function HomeMap(){
    map.easeTo({
      center: [-6.0025, 37.412],
      zoom: Math.max(map.getZoom(), 5),
      offset: [0, -1 / 2],
    });
  }

  function MissionMap(){
    map.easeTo({
      center: [MissionHome[1],MissionHome[0]],
      zoom: Math.max(map.getZoom(), 15),
      offset: [0, -1 / 2],
    });
  }

  
  function openAddUav(){
    SetAddUAVOpen(true);
  }

  
  const hideStatusWindow =() =>{
    sethidestatus(!hidestatus);
  }
  
  useEffect(() => {
    setMissionName(Mission_Name);
  }, [Mission_Name]);
  useEffect(() => {
    setMissionHome(Mission_Home);
    MissionMap();
  }, [Mission_Home]);
  

  return (

    <header className="toolbar toolbar-header">

      <div className="toolbar-actions">

        <ButtonGroup variant="contained" size="small" aria-label="outlined primary button group">
          <Button onClick={HomeMap}>
            <HomeIcon fontSize="small"  />
          </Button>
            <Button id="openMission" >
              <label htmlFor="openMissionNavbar" className="icon icon-folder" style={{padding: 0}} >
              <FolderIcon fontSize="small"  />
              </label>
              <input type="file" multiple={false} style={{display:"none"}} id="openMissionNavbar" onChange={readFile} />
            </Button>
            <Button id="openAddUav" onClick={openAddUav} >
              <FlightIcon fontSize="small"  />
            </Button>            
        </ButtonGroup>
        <RosContext.Consumer>
          {({rosState}) => (
          <Button id="rosConnect" onClick={rosContex.rosConnect} >{rosState && "conectado"} {!rosState && "desconectado"}  </Button>
          )}
        </RosContext.Consumer>
        
        <Button id="loadMission" size="small" variant="contained" onClick={openAddUav} >
              <FileUploadIcon fontSize="small"  />
        </Button>   

        <Button id="commandMission"  variant="contained" onClick={openAddUav} >
        Fly!
        </Button>   

        <Button id="openedmission" variant="contained" onClick={MissionMap} > {MissionName}</Button>
        <Button variant="filledTonal" onClick={e => hideStatusWindow() } >Status</Button>


        
        <Button id="openTerminal" className="btn btn-default" style={{visibility: true}}>
        <TabIcon fontSize="small"  />
        </Button>      
        <Button id="UndoMission" className="btn btn-default" style={{visibility: true}}>
        <ReplyIcon fontSize="small"  />
        </Button>

        <Button id="reset" className="btn btn-default pull-right">
        <CallMadeIcon fontSize="small"  />
        </Button>
          
        <Button id="showManualMode" className="btn btn-default pull-right">
        <ModeEditIcon fontSize="small"  />
        </Button>
          
        <Button id="commandManualMission" style={{visibility: true}} className="btn btn-default pull-right">
          <CachedIcon fontSize="small"  />
        </Button>
        
      </div>
      
  </header>



  )
}
