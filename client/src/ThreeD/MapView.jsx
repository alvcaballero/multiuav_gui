import { Canvas } from '@react-three/fiber';
import * as THREE from 'three';
import CameraControls from '../ThreeD/CameraControls';
import Pose from '../ThreeD/Pose';
import Polyhedron from '../ThreeD/Polyhedron';
import Drone from '../ThreeD/Drone';
import NumberedSphere from '../ThreeD/NumberedSphere';
import groundTextureImg from '../resources/3d/ground.jpg';

// Crear textura del suelo
const groundTexture = new THREE.TextureLoader().load(
    groundTextureImg
  );
  groundTexture.wrapS = groundTexture.wrapT = THREE.RepeatWrapping;
  groundTexture.repeat.set(25, 25);

const MapView = ({children}) => {
  return (
    <Canvas camera={{ position: [1, 2, 3] }}>
        {/* Suelo infinito */}
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.1, 0]}>
            <planeGeometry args={[1000, 1000]} />
            <meshStandardMaterial 
                map={groundTexture}
                side={THREE.DoubleSide}
                roughness={0.8}
                metalness={0.2}
            />
        </mesh>

        {/* Iluminación */}
        <ambientLight intensity={0.5} />
        <directionalLight position={[10, 10, 5]} intensity={1} />

        <CameraControls />
        <axesHelper args={[5]} />
        <gridHelper />
        {children}
    </Canvas>
  );
};

export default MapView  