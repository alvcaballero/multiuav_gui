import React, { useRef } from 'react';
import { extend, ReactThreeFiber, useFrame, useThree } from '@react-three/fiber'; // eslint-disable-line
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

// add OrbitControls as external three.js thing
extend({ OrbitControls });

export default function CameraControls(props) {
  // https://codesandbox.io/s/r3f-orbit-controls-un2oh?from-embed=&file=/src/index.js
  const ref = useRef();
  const { camera, gl } = useThree();
  useFrame(() => ref.current && ref.current.update());
  return (
    <orbitControls
      ref={ref}
      args={[camera, gl.domElement]}
      target={[0, 0, 0]}
      enableRotate={false}
      enableZoom={true}
      enableDamping
      dampingFactor={0.08}
      screenSpacePanning
      mouseButtons={{
        LEFT: THREE.MOUSE.PAN,
        MIDDLE: THREE.MOUSE.DOLLY,
        RIGHT: THREE.MOUSE.ROTATE,
      }}
      {...props}
    />
  );
}
