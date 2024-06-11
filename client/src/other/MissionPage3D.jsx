import React, { useState, useEffect, Fragment } from 'react';
import { useSelector } from 'react-redux';
import { Paper, Tab, Tabs } from '@mui/material';
import makeStyles from '@mui/styles/makeStyles';
import { Canvas } from '@react-three/fiber';
import maplibregl from 'maplibre-gl';

import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';

import { Navbar2 } from '../components/Navbar2';
import { Menu } from '../components/Menu';
import palette from '../common/palette';

import { RosControl } from '../components/RosControl';
import { MissionController } from '../components/MissionController';
import MissionPanel from '../components/MissionPanel';
import MissionElevation from '../components/MissionElevation';
import SaveFile from '../components/SaveFile';
import CameraControls from '../ThreeD/CameraControls';
import Pose from '../ThreeD/Pose';
import Polyhedron from '../ThreeD/Polyhedron';

const useStyles = makeStyles((theme) => ({
  root: {
    margin: '0',
    height: '100vh',
  },
  sidebarStyle: {
    display: 'flex',
    flexDirection: 'column',
    position: 'fixed',
    left: 0,
    top: '88px',
    height: 'calc(100% - 95px)',
    width: '560px',
    margin: '0px',
    zIndex: 3,
  },
  middleStyle: {
    flex: 1,
    display: 'grid',
  },
  panelElevation: {
    display: 'flex',
    flexDirection: 'column',
    position: 'fixed',
    right: 0,
    bottom: 0,
    height: '30vh',
    width: 'calc(100% - 560px)',
    margin: '0px',
    zIndex: 3,
  },
}));
const showToast = (type, description) => {
  setList([...list, toastProperties]);
};

const MissionPage3D = () => {
  const classes = useStyles();
  const tabIndex = 0;

  const [Opensave, setOpenSave] = useState(false);

  const positions = useSelector((state) => state.data.positions);
  const routes = useSelector((state) => state.mission.route);
  const [routeLines, setRouteLines] = useState([]);
  const [routeWp, setRouteWp] = useState();

  const [filteredPositions, setFilteredPositions] = useState([]);

  const polyhedron = [
    new THREE.BoxGeometry(),
    new THREE.SphereGeometry(0.785398),
    new THREE.DodecahedronGeometry(0.785398),
  ];

  const latLonToXYZ = (lat, lon, alt) => {
    const radius = 6371; // Earth radius in kilometers
    const phi = (90 - lat) * (Math.PI / 180);
    const theta = (lon + 180) * (Math.PI / 180);

    const x = -(radius + alt) * Math.sin(phi) * Math.cos(theta);
    const y = (radius + alt) * Math.cos(phi);
    const z = (radius + alt) * Math.sin(phi) * Math.sin(theta);

    return [x, y, z];
  };

  function calculateDistanceMercatorToMeters(from, to) {
    const mercatorPerMeter = from.meterInMercatorCoordinateUnits();
    // mercator x: 0=west, 1=east
    const dEast = to.x - from.x;
    const dEastMeter = dEast / mercatorPerMeter;
    // mercator y: 0=north, 1=south
    const dNorth = from.y - to.y;
    const dNorthMeter = dNorth / mercatorPerMeter;
    return { dEastMeter, dNorthMeter };
  }

  useEffect(() => {
    let line = [];
    let origen = null;
    let origen2 = null;
    let lineVector3 = [];
    let routeline = [];
    let routelineVector3 = [];

    routes.map((rt, index_rt) => {
      line = [];
      rt.wp.map((wp, index_wp) => {
        if (origen == null) {
          //origen = latLonToXYZ(wp['pos'][1], wp['pos'][0], 0);
          origen = [wp['pos'][1], wp['pos'][0], 0];
          origen2 = maplibregl.MercatorCoordinate.fromLngLat({ lng: wp['pos'][1], lat: wp['pos'][0] }, 0);
          let test2 = maplibregl.MercatorCoordinate.fromLngLat({ lng: wp['pos'][1], lat: wp['pos'][0] }, wp['pos'][2]);
          let test3 = calculateDistanceMercatorToMeters(origen2, test2);
          console.log(origen2);
          console.log(test2);
          console.log(test3);
        }
        let destino = maplibregl.MercatorCoordinate.fromLngLat({ lng: wp['pos'][1], lat: wp['pos'][0] }, wp['pos'][2]);
        let distance = calculateDistanceMercatorToMeters(origen2, destino);

        //let destino = latLonToXYZ(wp['pos'][1], wp['pos'][0], wp['pos'][2]);
        //line.push([destino[0] - origen[0], destino[1] - origen[1], destino[2] - origen[2]]);
        line.push([distance.dEastMeter, distance.dNorthMeter, wp['pos'][2] - origen[2]]);
      });
      routeline.push(line);
    });
    console.log(line);
    routeline.map((line) => {
      let mylineVector3 = [];
      line.map((point) => {
        mylineVector3.push(new THREE.Vector3(point[0], point[2], point[1]));
      });
      routelineVector3.push(new THREE.BufferGeometry().setFromPoints(mylineVector3));
    });
    line.map((point) => {
      lineVector3.push(new THREE.Vector3(point[0], point[2], point[1]));
    });
    //setRouteLines(new THREE.BufferGeometry().setFromPoints(lineVector3));
    setRouteLines(routelineVector3);
  }, [routes]);

  useEffect(() => {
    setFilteredPositions(Object.values(positions));
  }, [positions]);

  const tabs = (
    <>
      <Tabs value={tabIndex} onChange={(_, index) => navigate(`/robot/${id}/${index}`)} style={{ flexGrow: 1 }}>
        <Tab label="Viz" />
        <Tab label="Imagery" />
        <Tab label="Stats" />
        <Tab label="Report" />
      </Tabs>
    </>
  );

  return (
    <div className={classes.root}>
      <MissionController>
        <RosControl notification={showToast}>
          <Navbar2 tabs={tabs} />
          <Menu />
          <div
            style={{
              float: 'right',
              width: 'calc(100% - 560px)',
              height: 'calc(70vh - 95px)',
              right: '0px',
              margin: 'auto',
            }}
          >
            <Canvas camera={{ position: [1, 2, 3] }}>
              <Polyhedron position={[2, 2, 0]} polyhedron={polyhedron} />
              {React.Children.toArray(
                routeLines.map((line, index) => (
                  <Fragment key={'line' + index}>
                    <line geometry={line}>
                      <lineBasicMaterial
                        attach="material"
                        color={palette.colors_devices[index]}
                        linewidth={10}
                        linecap={'round'}
                        linejoin={'round'}
                      />
                    </line>
                  </Fragment>
                ))
              )}

              <mesh rotation-y={2}>
                <boxGeometry />
                <meshBasicMaterial color="orange" />
              </mesh>
              <mesh rotation={[-Math.PI / 2, 0, 0]}>
                <planeGeometry args={[5, 5, 64, 64]}></planeGeometry>
                <meshBasicMaterial attach="material" transparent side={THREE.DoubleSide}></meshBasicMaterial>
              </mesh>
              <Pose
                x={0}
                y={0}
                theta={0}
                materialProps={{
                  color: new THREE.Color(0x3287a8),
                  wireframe: true,
                }}
              />

              <OrbitControls />
              <axesHelper args={[5]} />
              <gridHelper />
            </Canvas>
          </div>

          <div className={classes.sidebarStyle}>
            <div className={classes.middleStyle}>
              <Paper square>
                <MissionPanel SetOpenSave={setOpenSave} />
              </Paper>
            </div>
          </div>
          <div className={classes.panelElevation}>
            <div className={classes.middleStyle}>
              <Paper square>
                <MissionElevation />
              </Paper>
            </div>
          </div>
          {Opensave && <SaveFile SetOpenSave={setOpenSave} />}
        </RosControl>
      </MissionController>
    </div>
  );
};

export default MissionPage3D;
