import { Canvas, useThree } from '@react-three/fiber';
import { Sky, Stats } from '@react-three/drei';
import React, { Suspense, useEffect } from 'react';
import * as THREE from 'three';
import { Perf } from 'r3f-perf';

import CameraControls from '../ThreeD/CameraControls';
import { groundTexture, waterTexture } from './textures';
// Crear textura del suelo
groundTexture.repeat.set(100, 100);

const Environment = () => {
  const { gl, scene } = useThree();
  scene.background = new THREE.Color('skyblue');
  scene.fog = new THREE.Fog('#abddff', 600, 2000);
  return null;
};

{
  /* Suelo infinito */
}
const GROUND_SIZE = 1000;
const WATER_SIZE = 8000;

// Área de tierra interior
const Ground = () => (
  <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.1, 0]}>
    <planeGeometry args={[GROUND_SIZE, GROUND_SIZE]} />
    <meshStandardMaterial map={groundTexture} roughness={0.8} metalness={0.2} />
  </mesh>
);

// Agua exterior con agujero donde está la tierra
const Water = () => {
  const geometry = React.useMemo(() => {
    const half = WATER_SIZE / 2;
    const halfG = GROUND_SIZE / 2;

    const shape = new THREE.Shape();
    shape.moveTo(-half, -half);
    shape.lineTo(half, -half);
    shape.lineTo(half, half);
    shape.lineTo(-half, half);
    shape.lineTo(-half, -half);

    // agujero cuadrado donde va la tierra
    const hole = new THREE.Path();
    hole.moveTo(-halfG, -halfG);
    hole.lineTo(-halfG, halfG);
    hole.lineTo(halfG, halfG);
    hole.lineTo(halfG, -halfG);
    hole.lineTo(-halfG, -halfG);
    shape.holes.push(hole);

    const geo = new THREE.ShapeGeometry(shape);
    // ShapeGeometry está en XY, la rotamos a XZ después
    // UV repeat manual para que la textura no se estire
    const uvAttribute = geo.attributes.uv;
    const posAttribute = geo.attributes.position;
    for (let i = 0; i < uvAttribute.count; i++) {
      uvAttribute.setXY(i, posAttribute.getX(i) / 16, posAttribute.getY(i) / 16);
    }
    uvAttribute.needsUpdate = true;
    return geo;
  }, []);

  return (
    <mesh geometry={geometry} rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.15, 0]}>
      <meshStandardMaterial map={waterTexture} roughness={0.2} metalness={0.1} color="#4aafd5" />
    </mesh>
  );
};

const MapView = ({ children }) => {
  return (
    <Canvas camera={{ position: [100, 100, 100], fov: 35, near: 2, far: 800 }}>
      {/* Iluminación */}
      <Sky sunPosition={[100, 100, 100]} />
      <fog attach="fog" args={['#abddff', 600, 2000]} />
      <ambientLight intensity={0.5} />
      <directionalLight position={[50, 50, 50]} intensity={3} />
      <Perf />

      <CameraControls />
      <axesHelper args={[5]} />
      <gridHelper />

      <Suspense fallback={null}>
        <Ground />
        <Water />
        {children}
      </Suspense>
    </Canvas>
  );
};

export default MapView;
