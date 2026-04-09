import React, { useEffect, useRef, useState } from 'react';
import { useFrame } from '@react-three/fiber';
import { useSelector } from 'react-redux';
import * as THREE from 'three';
import { getModelPath } from '../models/ModelLoader.jsx';
import { LatLon2XYZObj } from '../core/convertion';
import { useGLTF, useHelper } from '@react-three/drei';

const RING_HEIGHT_OFFSET = 1; // meters above drone

const Device = ({ id, position, isSelected, category }) => {
  const meshRef = useRef();
  const camRef = useRef();

  const currentPosition = useRef(new THREE.Vector3());
  const nextPosition = useRef(new THREE.Vector3());

  const model = useGLTF(getModelPath(category));

  useHelper(camRef, THREE.CameraHelper);

  const initialized = useRef(false);

  useEffect(() => {
    const loc = position.find((item) => item.deviceId == id);
    if (loc) {
      nextPosition.current.set(loc.pos[0], loc.pos[2], -loc.pos[1]);
      if (!initialized.current) {
        currentPosition.current.copy(nextPosition.current);
        initialized.current = true;
      }
      if (meshRef.current && loc.course !== undefined) {
        // course: 0=North, clockwise. Three.js Y-up: negate for correct direction.
        meshRef.current.rotation.y = -(loc.course * Math.PI) / 180;
      }
      if (camRef.current && loc.gimbalPitch !== undefined) {
        // gimbalPitch: 0=horizontal, -90=nadir. Negate to map to Three.js camera pitch.
        camRef.current.rotation.x = (loc.gimbalPitch * Math.PI) / 180;
      }
    }
  }, [position, id]);

  useFrame(() => {
    if (meshRef.current) {
      currentPosition.current.lerp(nextPosition.current, 0.07);
      meshRef.current.position.copy(currentPosition.current);
    }
  });

  if (!model) return null;

  return (
    <group ref={meshRef}>
      <primitive object={model.scene.clone()} scale={[1, 1, 1]} />
      {/* Camera helper: represents the drone's forward-looking camera direction */}
      <perspectiveCamera ref={camRef} fov={60} near={1} far={4} position={[0, 0, 0]} />
      {isSelected && (
        <mesh position={[0, RING_HEIGHT_OFFSET, 0]}>
          <boxGeometry args={[0.5, 0.5, 0.5]} />
          <meshBasicMaterial color="#00aaff" depthTest={false} depthWrite={false} />
        </mesh>
      )}
    </group>
  );
};

const R3FDevices = () => {
  const devices = useSelector((state) => state.devices.items);
  const positions = useSelector((state) => state.session.positions);
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const origin3d = useSelector((state) => state.session.scene3d.origin);
  const [positionxyz, setPositionxyz] = useState([]);

  useEffect(() => {
    const pos = Object.values(positions).map((item) => ({
      ...item,
      lng: item.longitude,
      lat: item.latitude,
      alt: item.altitude,
    }));
    const posxyz = LatLon2XYZObj(origin3d, pos, 1000);
    const result = posxyz.map((item) => ({
      ...item,
      name: devices[item.deviceId]?.name,
      course: positions[item.deviceId]?.course,
      gimbalPitch: positions[item.deviceId]?.attributes?.gimbal?.[0] ?? 0,
      gimbalYaw: positions[item.deviceId]?.attributes?.gimbal?.[1] ?? 0,
    }));
    setPositionxyz(result);
  }, [origin3d, positions]);

  return (
    <>
      {positionxyz.map((item) => (
        <Device
          key={item.deviceId}
          id={item.deviceId}
          position={positionxyz}
          isSelected={String(selectedDeviceId) === String(item.deviceId)}
          category={devices[item.deviceId]?.category}
        />
      ))}
    </>
  );
};

export default R3FDevices;
