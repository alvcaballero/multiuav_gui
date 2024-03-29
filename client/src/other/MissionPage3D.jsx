import React, { useState, useEffect } from 'react';
import { useSelector } from 'react-redux';
import { Paper, Tab, Tabs } from '@mui/material';
import makeStyles from '@mui/styles/makeStyles';
import { Canvas } from '@react-three/fiber';

import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';

import { Navbar2 } from '../components/Navbar2';
import { Menu } from '../components/Menu';

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

  const [filteredPositions, setFilteredPositions] = useState([]);

  const polyhedron = [
    new THREE.BoxGeometry(),
    new THREE.SphereGeometry(0.785398),
    new THREE.DodecahedronGeometry(0.785398),
  ];

  useEffect(() => {
    setFilteredPositions(Object.values(positions));
  }, [positions]);

  const tabs = (
    <>
      <Tabs
        value={tabIndex}
        onChange={(_, index) => navigate(`/robot/${id}/${index}`)}
        style={{ flexGrow: 1 }}
      >
        <Tab label="Viz" />
        <Tab label="Imagery" />
        <Tab label="Stats" />
        <Tab label="Report" />
      </Tabs>
    </>
  );
  const points = [];
  points.push(new THREE.Vector3(-10, 0, 0));
  points.push(new THREE.Vector3(0, 10, 0));
  points.push(new THREE.Vector3(10, 0, 0));

  const lineGeometry = new THREE.BufferGeometry().setFromPoints(points);

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
              <line geometry={lineGeometry}>
                <lineBasicMaterial
                  attach="material"
                  color={'#9c88ff'}
                  linewidth={10}
                  linecap={'round'}
                  linejoin={'round'}
                />
              </line>
              <mesh rotation-y={2}>
                <boxGeometry />
                <meshBasicMaterial color="orange" />
              </mesh>
              <mesh rotation={[-Math.PI / 2, 0, 0]}>
                <planeGeometry args={[5, 5, 64, 64]}></planeGeometry>
                <meshBasicMaterial
                  attach="material"
                  transparent
                  side={THREE.DoubleSide}
                ></meshBasicMaterial>
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
