import React from 'react'

export const Overlaytab = () => {


    function hideRoster() {
        var x = document.getElementById("uavTable");
        if (x.style.display === "block") {
          x.style.display = "none";
        } else {
          x.style.display = "block";
        }
    }

  return (

    <div className="overlayTab">
        <div id="myitem">
            <span> UAV Roster</span>
            <button id="hideRosterButton" onClick={hideRoster} className="btn btn-dafault hideRosterBtn">
                <span className="icon icon-eye"></span>                        
            </button>

            <div style = {{overflowX:'auto' }}>
                    
                <table id="uavTable">     
                <tbody>       
                    <tr>
                        <th>Name</th>
                        <th>Altitude</th>
                        <th>Velocity</th>
                        <th>Battery</th>
                        <th>Mission</th>
                        <th>Info</th>
                    </tr>
                    <tr>
                        <td>No uavs</td>
                        <td>0</td>
                        <td>0</td>
                        <td>0</td>
                        <td>
                        <button className="btn btn-default icon-front">
                        <span className="icon icon-upload"></span>
                        </button><button className = "btn btn-default icon-front">Fly!</button>
                        </td>
                        <td>No info</td>
                    </tr>
                    </tbody>
                </table>
            </div>
        </div>
  </div>
  )
}
