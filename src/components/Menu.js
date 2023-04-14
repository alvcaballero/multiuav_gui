import { Height } from '@mui/icons-material';
import React ,{ useEffect,useState ,useContext }from 'react'
import { map } from '../Mapview/Mapview.js'
import { RosContext } from './RosControl'
import { useSelector } from 'react-redux';

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
  }, [Mission_Home]);
  

  return (

    <header className="toolbar toolbar-header">

      <div className="toolbar-actions">
        <div className="btn-group">
            <button id="setHome" onClick={HomeMap} className="btn btn-default">
              <span className="icon icon-home"></span>
            </button>
            <button id="openMission" className="btn btn-default" >
              <label htmlFor="openMissionNavbar" className="icon icon-folder" style={{padding: 0}} ></label>
              <input type="file" multiple={false} style={{display:"none"}} id="openMissionNavbar" onChange={readFile} />
            </button>
            <button id="openAddUav" onClick={openAddUav} className="btn btn-default pull-right">
              <span className="icon icon-flight"></span>
            </button>            
        </div>
        <RosContext.Consumer>
          {({rosState}) => (
          <button id="rosConnect" onClick={rosContex.rosConnect} className="btn btn-default">{rosState && "conectado"} {!rosState && "desconectado"}  </button>
          )}
        </RosContext.Consumer>
        <button id="loadMission" style={{visibility: true}} className="btn btn-default">
          <span className="icon icon-upload"></span>
        </button>

        <button id="commandMission" style={{visibility: true}} className="btn btn-default">Fly!</button>

        <div id="openedmission" onClick={MissionMap} className="btn btn-default"> {MissionName}</div>
        <button className="btn btn-default pull-right" onClick={e => hideStatusWindow() } >Status</button>


        
        <button id="openTerminal" className="btn btn-default" style={{visibility: true}}>
          <span className="icon icon-window"></span>
        </button>      
        <button id="UndoMission" className="btn btn-default" style={{visibility: true}}>
          <span className="icon icon-reply"></span>
        </button>

        <button id="reset" className="btn btn-default pull-right">
          <span className="icon icon-retweet"></span>
        </button>
          
        <button id="showManualMode" className="btn btn-default pull-right">
          <span className="icon icon-pencil"></span>
        </button>
          
        <button id="commandManualMission" style={{visibility: true}} className="btn btn-default pull-right">
          <span className="icon icon-direction"></span>
        </button>
        
      </div>
      
  </header>



  )
}
