import { Canvas,useThree } from '@react-three/fiber';
import { Sky } from '@react-three/drei'
import React, { Suspense,useEffect } from 'react';
import * as THREE from 'three';
import CameraControls from '../ThreeD/CameraControls';
import { groundTexture } from './textures';
// Crear textura del suelo
groundTexture.repeat.set(100, 100);

const Environment = () => {
  const { gl,scene } = useThree();
  scene.background = new THREE.Color("skyblue");
  scene.fog = new THREE.Fog("#abddff", 400, 800);
  return null;
}


const MapView = ({ children }) => {

  return (
    <Canvas camera={{  position: [10, 10, 5], fov: 35, near: 2, far: 800 }} >
      {/* Iluminación */}
      <Sky sunPosition={[100, 100, 100]} />
      <fog attach="fog" args={['#abddff', 500, 800]} />
      <ambientLight intensity={0.5} />
      <directionalLight position={[50, 50, 50]} intensity={3} />

      <CameraControls />
      <axesHelper args={[5]} />
      <gridHelper />

      <Suspense fallback={null}>
        {children}
      </Suspense>
      {/* Suelo infinito */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.1, 0]}>
        <planeGeometry args={[1000, 1000]} />
        <meshStandardMaterial
          map={groundTexture}
          roughness={0.8}
          metalness={0.2}
        />
      </mesh>


    </Canvas>
  );
};

export default MapView  