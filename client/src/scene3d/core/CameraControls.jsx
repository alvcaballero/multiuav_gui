import { useEffect, useRef } from 'react';
import { useThree, useFrame, invalidate as r3fInvalidate } from '@react-three/fiber';
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

  // Target position for smooth follow — updated on each telemetry tick, consumed by useFrame lerp
  const followTargetRef = useRef(null);

  useEffect(() => {
    if (!mapFollow || !followPosition?.latitude) return;
    const alt = followPosition.attributes?.home
      ? followPosition.altitude - followPosition.attributes.home[2]
      : (followPosition.altitude ?? 0);
    const xyz = LatLon2XYZ(origin3d, {
      lng: followPosition.longitude ?? origin3d.lng,
      lat: followPosition.latitude ?? origin3d.lat,
      alt,
    });
    followTargetRef.current = new THREE.Vector3(xyz[0], alt, -xyz[1]);
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

  // Reusable vectors to avoid per-frame allocation.
  const _dir = useRef(new THREE.Vector3());
  const _offset = useRef(new THREE.Vector3());

  useFrame((_, delta) => {
    if (!controlsRef.current) return;

    const controls = controlsRef.current;
    let didMove = false;

    if (mapFollow && followTargetRef.current) {
      const lerpFactor = 1 - Math.exp(-8 * delta);
      _offset.current.copy(camera.position).sub(controls.target);
      controls.target.lerp(followTargetRef.current, lerpFactor);
      camera.position.copy(controls.target).add(_offset.current);
      controls.update();
      didMove = true;
    }

    _dir.current.set(0, 0, 0);
    if (keys.current.w || keys.current.ArrowUp) _dir.current.z -= moveSpeed;
    if (keys.current.s || keys.current.ArrowDown) _dir.current.z += moveSpeed;
    if (keys.current.a || keys.current.ArrowLeft) _dir.current.x -= moveSpeed;
    if (keys.current.d || keys.current.ArrowRight) _dir.current.x += moveSpeed;
    if (keys.current.q) _dir.current.y += moveSpeed;
    if (keys.current.e) _dir.current.y -= moveSpeed;

    if (_dir.current.lengthSq() > 0) {
      _dir.current.applyQuaternion(camera.quaternion);
      const newY = camera.position.y + _dir.current.y;

      if (newY >= minHeight) {
        camera.position.add(_dir.current);
        controls.target.add(_dir.current);
      } else {
        _dir.current.y = 0;
        camera.position.add(_dir.current);
        controls.target.add(_dir.current);
        camera.position.y = minHeight;
        controls.target.y = minHeight;
      }
      controls.update();
      didMove = true;
    }

    _offset.current.copy(camera.position).sub(controls.target);
    const horizontalDist = Math.sqrt(
      _offset.current.x * _offset.current.x + _offset.current.z * _offset.current.z,
    );
    const bearing = Math.atan2(_offset.current.x, -_offset.current.z) * (180 / Math.PI);
    const pitch = Math.atan2(_offset.current.y, horizontalDist) * (180 / Math.PI);

    const prev = lastAzimuthRef.current;
    if (!prev || Math.abs(bearing - prev.bearing) > 0.3 || Math.abs(pitch - prev.pitch) > 0.3) {
      lastAzimuthRef.current = { bearing, pitch, roll: 0 };
      window.dispatchEvent(
        new CustomEvent('camera-azimuth', { detail: { bearing, pitch, roll: 0 } }),
      );
      didMove = true;
    }

    if (didMove) r3fInvalidate();
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
