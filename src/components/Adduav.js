import React,{ useContext } from 'react'
import { RosContext } from './RosControl'
import {Select, MenuItem } from "@mui/material"

export const Adduav = ({SetAddUAVOpen}) => {
    const rosContex = useContext(RosContext);

    const sidenavStyle={
        height: '100%', /* 100% Full-height */
        width: '260px', /* 0 width - change this with JavaScript */
        Zindex: '1', /* Stay on top */
        left: '0',
        overflowX:'hidden',/* Disable horizontal scroll */
        transition: '0.5s' /* 0.5 second transition effect to slide in the sidenav */
      }

	function showAddUav(){
		if(document.getElementById("AddUav").style.width === "250px"){
			document.getElementById("AddUav").style.width = "0";
		}else{
			document.getElementById("AddUav").style.width = "250px";
		}
	}

    function closeAddUav(){
        SetAddUAVOpen(false);

    }
    function AddnewUAV(){
        let uav_ns = document.getElementById("UAV_NS").value;
        let uav_type_idx = document.getElementById("UAV_options").selectedIndex;
        let uav_type = document.getElementById("UAV_options").options[uav_type_idx].value;
        rosContex.connectAddUav(uav_ns,uav_type)
        SetAddUAVOpen(false);

    }
    function disConnectUav (){
      SetAddUAVOpen(false);
    }


  return (

    <div className="overlay">
        <div className="window-content">
          <div id="AddUav" style={sidenavStyle}>
            <nav className="nav-group">
              <div className="container">
                <span><b>Uav Platform Login</b></span>
                  <div className="row">
                    <div className="col-25">
                      <label>UAV Name</label>
                    </div>
                    <div className="col-75">
                       <input type="text" id="UAV_NS" placeholder="UAV name"/>                    
                    </div>
                  </div>
                  <div className="row" style={{display:"inline-block"}}>
                    <div className="col-25">
                      <label htmlFor="uavtype">Uav Type    </label>
                    </div>
                    <div className="col-75" style={{width:"125px"}}>
                      <select id="UAV_options" name="UAV Type">
                        <option value="dji">DJI</option>
                        <option value="px4">PX4</option>
                        <option value="fuvex">Fuvex</option>
                        <option value="catec">Catec</option>                        
                      </select>
                    </div>
                  </div>
                  <div className="row">
                    <div className="form-btn-size">
                      <button style={{fontFamily : "inherit"}} id="closeAddUav" onClick={closeAddUav} className="btn btn-dafault">Close</button>
                      <button style={{fontFamily : "inherit"}} id="connectAddUav" onClick={AddnewUAV} className="btn btn-primary">Connect</button>
                    </div>
                  </div>                    
              </div>                  
            </nav>
          </div>
        </div>
      </div>
  )
}