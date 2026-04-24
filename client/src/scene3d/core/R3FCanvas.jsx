import { Canvas } from '@react-three/fiber';
import { Sky, Environment } from '@react-three/drei';
import React, { Suspense, useRef } from 'react';
import * as THREE from 'three';

import CameraControls from './CameraControls';
import OrientationGizmo from '../controls/OrientationGizmo';
import { groundTexture, waterTexture } from './textures';
import useOnlineStatus from './useOnlineStatus';
import MapTileGround from './MapTileGround';
import MapVectorGround from './MapVectorGround';
import useMartinStatus from './useMartinStatus';
// Crear textura del suelo
groundTexture.repeat.set(100, 100);

// GROUND_SIZE must match scene3d.range in the Redux store (default 1000m)
const GROUND_SIZE = 1000;
// WATER_SIZE covers the visual horizon beyond the ground patch
const WATER_SIZE = GROUND_SIZE * 8;
// Camera far plane and fog end should match to avoid visible cutoff
const CAMERA_FAR = 2000;
const FOG_NEAR = 600;
const FOG_FAR = CAMERA_FAR;

// Área de tierra interior + agua exterior (fallback offline)
const Ground = () => (
  <>
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.1, 0]} receiveShadow>
      <planeGeometry args={[GROUND_SIZE, GROUND_SIZE]} />
      <meshStandardMaterial map={groundTexture} roughness={0.8} metalness={0.2} />
    </mesh>
    <Water />
  </>
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

const R3FCanvas = ({ children }) => {
  const online = useOnlineStatus();
  const martinAvailable = useMartinStatus();
  const controlsRef = useRef();

  return (
    <Canvas
      shadows
      gl={{ antialias: true, toneMapping: THREE.ACESFilmicToneMapping, toneMappingExposure: 0.9 }}
      camera={{ position: [100, 100, 100], fov: 35, near: 2, far: CAMERA_FAR }}
    >
      {/* Iluminación */}
      <Sky sunPosition={[1, 0.4, 0]} rayleigh={0.8} turbidity={8} />
      <Environment preset="sunset" />
      <fog attach="fog" args={['#cce0f0', FOG_NEAR, FOG_FAR]} />
      <ambientLight intensity={0.4} />
      <hemisphereLight args={['#87ceeb', '#4a7c59', 0.6]} />
      <directionalLight
        position={[200, 400, 100]}
        intensity={2.5}
        castShadow
        shadow-mapSize={[2048, 2048]}
        shadow-camera-far={1000}
        shadow-camera-left={-500}
        shadow-camera-right={500}
        shadow-camera-top={500}
        shadow-camera-bottom={-500}
      />

      <CameraControls controlsRef={controlsRef} />
      <OrientationGizmo controlsRef={controlsRef} />
      <axesHelper args={[5]} />
      <gridHelper />

      <Suspense fallback={null}>
        {martinAvailable ? (
          <>
            <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.05, 0]}>
              <planeGeometry args={[GROUND_SIZE * 8, GROUND_SIZE * 8]} />
              <meshBasicMaterial color="#ffffff" />
            </mesh>
            <MapVectorGround />
          </>
        ) : online ? (
          <MapTileGround />
        ) : (
          <Ground />
        )}
        {children}
      </Suspense>
    </Canvas>
  );
};

export default R3FCanvas;
