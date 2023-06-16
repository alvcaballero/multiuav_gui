import './MainPage.css';
import React from 'react'
import MapView from './Mapview/Mapview'
import { Navbar } from './components/Navbar';
import MapMissionsCreate from './Mapview/MapMissionsCreate';


const MissionPage = () => {
  return (
    <div >
      <div >
        <Navbar  />

        <MapView>
          <MapMissionsCreate></MapMissionsCreate>


        </MapView>
      </div>
    </div>
  )
}

export default MissionPage