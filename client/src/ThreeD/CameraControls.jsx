import { useEffect, useRef } from 'react';
import { useThree, useFrame } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';

export default function CameraControls() {
  const { camera } = useThree();
  const controlsRef = useRef();
  const moveSpeed = 0.1;
  const minHeight = 0.1; // Minimum height above ground
  const keys = useRef({
    w: false,
    a: false,
    s: false,
    d: false,
    q: false, // Move up
    e: false, // Move down
    ArrowUp: false,
    ArrowDown: false,
    ArrowLeft: false,
    ArrowRight: false,
  });

  useEffect(() => {
    const handleKeyDown = (event) => {
      // For WASD keys, convert to lowercase
      const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
      if (keys.current.hasOwnProperty(key)) {
        keys.current[key] = true;
      }
    };

    const handleKeyUp = (event) => {
      // For WASD keys, convert to lowercase
      const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
      if (keys.current.hasOwnProperty(key)) {
        keys.current[key] = false;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  useFrame(() => {
    if (!controlsRef.current) return;

    const controls = controlsRef.current;
    const direction = new THREE.Vector3();

    // Forward/backward movement (W/S or up/down arrows)
    if (keys.current.w || keys.current.ArrowUp) {
      direction.z -= moveSpeed;
    }
    if (keys.current.s || keys.current.ArrowDown) {
      direction.z += moveSpeed;
    }

    // Left/right movement (A/D or left/right arrows)
    if (keys.current.a || keys.current.ArrowLeft) {
      direction.x -= moveSpeed;
    }
    if (keys.current.d || keys.current.ArrowRight) {
      direction.x += moveSpeed;
    }

    // Height control (Q/E)
    if (keys.current.q) {
      direction.y += moveSpeed;
    }
    if (keys.current.e) {
      direction.y -= moveSpeed;
    }

    // Apply movement to camera
    if (direction.length() > 0) {
      direction.applyQuaternion(camera.quaternion);
      
      // Calculate new position
      const newPosition = camera.position.clone().add(direction);
      const newTarget = controls.target.clone().add(direction);
      
      if (newPosition.y >= minHeight) {
        // If new position is above ground, apply complete movement
        camera.position.copy(newPosition);
        controls.target.copy(newTarget);
      } else {
        // If new position is below ground, maintain horizontal position
        // but adjust height to minimum allowed
        camera.position.set(newPosition.x, minHeight, newPosition.z);
        controls.target.set(newTarget.x, minHeight, newTarget.z);
      }
      
      controls.update();
    }
  });

  return (
    <OrbitControls
      ref={controlsRef}
      camera={camera}
      enableDamping
      dampingFactor={0.05}
      rotateSpeed={0.5}
      zoomSpeed={1}
      panSpeed={1}
      minDistance={1}
      maxDistance={100}
      minPolarAngle={0}
      maxPolarAngle={Math.PI / 2}
      minY={minHeight} // Minimum height restriction for target
    />
  );
}
