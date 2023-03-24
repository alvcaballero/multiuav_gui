import React, { useState, useContext } from 'react'
import "./Navbar.css"
import { map } from '../Mapview/Mapview.js'
import { RosContext } from './RosControl'


export const Navbar = ({SetAddUAVOpen}) => {
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

  function sethome(){
    map.easeTo({
      center: [-6.0025, 37.412],
      zoom: Math.max(map.getZoom(), 5),
      offset: [0, -1 / 2],
    });
  }
  
  function openAddUav(){
    SetAddUAVOpen(true);
  }
  

  return (
    <div className="navbar">
      <a id="homeNavbar" onClick={sethome}>Home</a>
      <div className="dropdown">
        <button className="dropbtn" >Ros </button>
        <div className="dropdown-content">
          <a id="rosConnectNavbar" onClick={rosContex.rosConnect}>Connect Rosbridge Server</a>                    
        </div>
      </div>     
      <div className="dropdown">
        <button className="dropbtn">UAV
          <i className="fa fa-caret-down"></i>
        </button>
        <div className="dropdown-content">
          <a id="openAddUavNavbar" onClick={openAddUav} >Connect UAV</a>
          <a id="hideRosterNavbar">Show/Hide UAV Roster</a>          
        </div>
      </div>
      <div className="dropdown">
        <button className="dropbtn">File
          <i className="fa fa-caret-down"></i>
        </button>
        <div className="dropdown-content">
          <label htmlFor="openMissionNavbar" >Abrir Mision</label>
          <input type="file" multiple={false} style={{display:"none"}} id="openMissionNavbar" onChange={readFile} />
          <a id="openKMLNavbar"  >Open Pylons & Wires File</a>
          <a id="resetNavbar"  >Reset Markers</a>          
        </div>
      </div>
      <div className="dropdown">
        <button className="dropbtn">Mission
          <i className="fa fa-caret-down"></i>
        </button>
        <div className="dropdown-content">
          <a id="loadMissionNavbar" onClick={rosContex.loadMission} >Load Mission</a>
          <a id="commandMissionNavbar" onClick={rosContex.commandMission} >Command Mission</a>          
        </div>
      </div>
      <a id="more info" >new device </a>
    </div>
  )
}
