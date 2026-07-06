import React, { useEffect, useRef, useState } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { useSelector } from 'react-redux';
import * as THREE from 'three';
import { getModelPath } from '../models/ModelLoader.jsx';
import { LatLon2XYZObj } from '../core/convertion';
import { useGLTF, useHelper } from '@react-three/drei';

const RING_HEIGHT_OFFSET = 1; // meters above drone

// Reusable objects to avoid per-frame allocations.
const _qYaw = new THREE.Quaternion();
const _qPitch = new THREE.Quaternion();
const _axisY = new THREE.Vector3(0, 1, 0);
const _axisX = new THREE.Vector3(1, 0, 0);

const Device = ({ id, position, isSelected, category }) => {
  const meshRef = useRef();
  const camRef = useRef();
  const { invalidate } = useThree();

  const currentPosition = useRef(new THREE.Vector3());
  const nextPosition = useRef(new THREE.Vector3());

  const model = useGLTF(getModelPath(category));

  useHelper(camRef, THREE.CameraHelper);

  // Boost envMapIntensity so PBR materials pick up the scene environment.
  useEffect(() => {
    if (!meshRef.current) return;
    meshRef.current.traverse((child) => {
      if (child.isMesh) {
        const mats = Array.isArray(child.material) ? child.material : [child.material];
        mats.forEach((m) => {
          if (m.isMeshStandardMaterial) {
            m.envMapIntensity = 1.2;
            m.needsUpdate = true;
          }
        });
      }
    });
  }, [model]);

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
        meshRef.current.rotation.y = -(loc.course * Math.PI) / 180;
      }
      if (camRef.current && loc.gimbalPitch !== undefined) {
        const gimbalYawRad =
          loc.gimbalYaw !== undefined ? (-(loc.gimbalYaw - loc.course) * Math.PI) / 180 : 0;
        const pitchRad = (loc.gimbalPitch * Math.PI) / 180;
        _qYaw.setFromAxisAngle(_axisY, gimbalYawRad);
        _qPitch.setFromAxisAngle(_axisX, pitchRad);
        camRef.current.quaternion.copy(_qYaw.multiply(_qPitch));
      }
      invalidate();
    }
  }, [position, id]);

  useFrame(() => {
    if (meshRef.current) {
      currentPosition.current.lerp(nextPosition.current, 0.07);
      meshRef.current.position.copy(currentPosition.current);
      // Keep requesting frames while the drone is still moving toward target.
      const dist = currentPosition.current.distanceToSquared(nextPosition.current);
      if (dist > 0.0001) invalidate();
    }
  });

  if (!model) return null;

  return (
    <group ref={meshRef}>
      <perspectiveCamera ref={camRef} fov={60} near={1} far={4} position={[0, 0.15, -0.15]} />
      <primitive object={model.scene.clone()} scale={[1, 1, 1]} />
      {/* Camera helper: represents the drone's forward-looking camera direction */}
      {isSelected && (
        <mesh position={[0, RING_HEIGHT_OFFSET, 0]}>
          <boxGeometry args={[0.2, 0.2, 0.2]} />
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
      lng: item.hasOwnProperty('longitude') ? item.longitude : origin3d.lng,
      lat: item.hasOwnProperty('latitude') ? item.latitude : origin3d.lat,
      alt: item.attributes?.home ? item.altitude - item.attributes.home[2] : (item.altitude ?? 0),
    }));
    const posxyz = LatLon2XYZObj(origin3d, pos, 1000);
    const result = posxyz.map((item) => ({
      ...item,
      name: devices[item.deviceId]?.name,
      course: positions[item.deviceId]?.course,
      gimbalPitch: positions[item.deviceId]?.attributes?.gimbal?.[0] ?? 0,
      gimbalYaw: positions[item.deviceId]?.attributes?.gimbal?.[2] ?? 0,
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
