import { Height } from '@mui/icons-material';
import React ,{ useState ,useContext }from 'react'
import { map } from '../Mapview/Mapview.js'
import { RosContext } from './RosControl'

export const Menu = ({SetAddUAVOpen}) => {
  const rosContex = useContext(RosContext);

  const [myValue, setMyValue] = useState('');


  const readFile = ( e ) => {
    //https://www.youtube.com/watch?v=K3SshoCXC2g
    const file = e.target.files[0];
    if ( !file ) return;

    const fileReader = new FileReader();
    fileReader.readAsText( file );
    fileReader.onload = () => {
      console.log( fileReader.result );
      console.log( file.name );
      setMyValue( fileReader.result );
      rosContex.openMision(fileReader.result)
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

  function openAddUav(){
    SetAddUAVOpen(true);
  }

    const [hidestatus,sethidestatus]  = useState(true);
    const hideStatusWindow =() =>{
      sethidestatus(!hidestatus);
    }

  return (

    <header className="toolbar toolbar-header">

      <div className="toolbar-actions">
        <div className="btn-group">
            <button id="setHome" onClick={HomeMap} className="btn btn-default">
              <span className="icon icon-home"></span>
            </button>
            <button id="openMission" className="btn btn-default">
              <label htmlFor="openMissionNavbar" className="icon icon-folder"></label>
              <input type="file" multiple={false} style={{display:"none"}} id="openMissionNavbar" onChange={readFile} />
            </button>
            <button id="openAddUav" onClick={openAddUav} className="btn btn-default pull-right">
              <span className="icon icon-flight"></span>
            </button>            
        </div>
        
        <button id="rosConnect" onClick={rosContex.rosConnect} className="btn btn-default">Connect ROS</button>
        <button id="loadMission" style={{visibility: true}} className="btn btn-default">
          <span className="icon icon-upload"></span>
        </button>

        <button id="commandMission" style={{visibility: true}} className="btn btn-default">Fly!</button>

        <div id="openedmission" className="btn btn-default"> No mission load </div>
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
