import React from 'react';
import * as THREE from 'three';

export default function Drone({ position = [0, 0, 0], rotation = [0, 0, 0], scale = 1, color = 0x3287a8 }) {
  return (
    <group position={position} rotation={rotation} scale={scale}>
      {/* Cuerpo principal */}
      <mesh>
        <boxGeometry args={[0.5, 0.5, 0.1]} />
        <meshBasicMaterial color={color} />
      </mesh>

      {/* Brazos del dron */}
      {[0, 90, 180, 270].map((angle, index) => (
        <group key={index} rotation={[0, 0, (angle * Math.PI) / 180]}>
          <mesh position={[0.4, 0, 0]}>
            <boxGeometry args={[0.8, 0.1, 0.1]} />
            <meshBasicMaterial color={color} />
          </mesh>
          {/* Motores */}
          <mesh position={[0.8, 0, 0]}>
            <cylinderGeometry args={[0.1, 0.1, 0.1, 16]} />
            <meshBasicMaterial color={0x333333} />
          </mesh>
        </group>
      ))}

      {/* Luces LED */}
      {[0, 90, 180, 270].map((angle, index) => (
        <mesh
          key={`led-${index}`}
          position={[
            Math.cos((angle * Math.PI) / 180) * 0.3,
            Math.sin((angle * Math.PI) / 180) * 0.3,
            0.05,
          ]}
        >
          <sphereGeometry args={[0.02, 16, 16]} />
          <meshBasicMaterial color={index < 2 ? 0xff0000 : 0x00ff00} />
        </mesh>
      ))}
    </group>
  );
} 