import { useEffect, useRef } from 'react';
import { useThree, useFrame } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import { useSelector } from 'react-redux';
import * as THREE from 'three';
import { LatLon2XYZ } from './convertion';

export default function CameraControls({ controlsRef: externalRef }) {
  const { camera } = useThree();
  const internalRef = useRef();
  const controlsRef = externalRef ?? internalRef;
  const moveSpeed = 1;
  const lastAzimuthRef = useRef(null);

  useEffect(() => {
    const handleOrientNorth = () => {
      if (!controlsRef.current) return;
      const controls = controlsRef.current;
      const target = controls.target.clone();
      const offset = camera.position.clone().sub(target);
      const horizontalDist = Math.sqrt(offset.x * offset.x + offset.z * offset.z);
      // Place camera south of target (positive Z) so it looks toward North (-Z)
      camera.position.set(target.x, camera.position.y, target.z + horizontalDist);
      controls.update();
    };

    const handleTopView = () => {
      if (!controlsRef.current) return;
      const controls = controlsRef.current;
      const target = controls.target.clone();
      const dist = camera.position.clone().sub(target).length();
      // Place camera directly above target looking straight down
      camera.position.set(target.x, target.y + dist, target.z);
      controls.update();
    };

    const handleZoomIn = () => {
      if (!controlsRef.current) return;
      const controls = controlsRef.current;
      const offset = camera.position.clone().sub(controls.target);
      camera.position.copy(controls.target).addScaledVector(offset, 0.7);
      controls.update();
    };

    const handleZoomOut = () => {
      if (!controlsRef.current) return;
      const controls = controlsRef.current;
      const offset = camera.position.clone().sub(controls.target);
      camera.position.copy(controls.target).addScaledVector(offset, 1.4);
      controls.update();
    };

    window.addEventListener('camera-orient-north', handleOrientNorth);
    window.addEventListener('camera-top-view', handleTopView);
    window.addEventListener('camera-zoom-in', handleZoomIn);
    window.addEventListener('camera-zoom-out', handleZoomOut);
    return () => {
      window.removeEventListener('camera-orient-north', handleOrientNorth);
      window.removeEventListener('camera-top-view', handleTopView);
      window.removeEventListener('camera-zoom-in', handleZoomIn);
      window.removeEventListener('camera-zoom-out', handleZoomOut);
    };
  }, [camera]);

  const mapFollow = useSelector((state) => state.devices.follow);
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const followPosition = useSelector((state) => state.session.positions[selectedDeviceId]);
  const origin3d = useSelector((state) => state.session.scene3d.origin);

  useEffect(() => {
    if (!mapFollow || !controlsRef.current || !followPosition?.latitude) return;
    const alt = followPosition.attributes?.home
      ? followPosition.altitude - followPosition.attributes.home[2]
      : followPosition.altitude;
    const xyz = LatLon2XYZ(origin3d, { lng: followPosition.longitude, lat: followPosition.latitude, alt });
    const controls = controlsRef.current;
    const offset = camera.position.clone().sub(controls.target);
    controls.target.set(xyz[0], alt, -xyz[1]);
    camera.position.copy(controls.target).add(offset);
    controls.update();
  }, [mapFollow, followPosition?.latitude, followPosition?.longitude, followPosition?.altitude]);
  const minHeight = 1; // Minimum height above ground
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

    // Emit bearing/pitch/roll so NorthOrientButton can replicate MapLibre's compass transform.
    // bearing: clockwise angle from North (-Z axis in ENU/Three.js), in degrees.
    // pitch: vertical tilt angle (0 = top-down, 90 = horizontal), in degrees.
    // roll: currently 0 (OrbitControls doesn't roll), reserved for future use.
    const offset = camera.position.clone().sub(controls.target);
    const horizontalDist = Math.sqrt(offset.x * offset.x + offset.z * offset.z);
    const bearingRad = Math.atan2(offset.x, -offset.z);
    const pitchRad = Math.atan2(offset.y, horizontalDist);
    const bearing = bearingRad * (180 / Math.PI);
    const pitch = pitchRad * (180 / Math.PI);

    const prev = lastAzimuthRef.current;
    if (!prev || Math.abs(bearing - prev.bearing) > 0.3 || Math.abs(pitch - prev.pitch) > 0.3) {
      lastAzimuthRef.current = { bearing, pitch, roll: 0 };
      window.dispatchEvent(new CustomEvent('camera-azimuth', { detail: { bearing, pitch, roll: 0 } }));
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
