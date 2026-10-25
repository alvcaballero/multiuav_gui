import React, { useEffect, useMemo, useRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { useSelector } from 'react-redux';
import * as THREE from 'three';
import { getModelPath } from '../models/ModelLoader.jsx';
import { LatLon2XYZObj } from '../core/convertion';
import { useGLTF, useHelper } from '@react-three/drei';
import R3FLidarCloud from '../lidar/R3FLidarCloud';

const RING_HEIGHT_OFFSET = 1; // meters above drone

// Positions arrive in batches (server WS_POSITIONS_INTERVAL_MS, 2 Hz by default). Each new sample is
// reached at CONSTANT speed (snapshot interpolation): the drone is drawn about one interval behind, but
// without the speed-up/slow-down that easing toward every sample produced (the "jumps" seen when flying fast).
const MIN_SAMPLE_GAP_MS = 50;
const MAX_SAMPLE_GAP_MS = 1000; // a drone that starts moving after standing still
// The motion lasts the AVERAGE gap times a margin, so the drone never reaches a sample before the next one
// arrives: lasting exactly the last gap, a sample 10 ms late stopped it for a frame twice per second, which
// the camera's follow turned into the whole world freezing. The margin costs (margin - 1) × gap of latency.
const SAMPLE_GAP_SMOOTHING = 0.2; // weight of the newest gap in the average
const PLAYOUT_MARGIN = 1.2;
// After OrbitControls' update (-1) and before CameraControls (0): the camera follows where the drone is
// drawn in THIS frame, not in the previous one.
const DEVICE_FRAME_PRIORITY = -0.5;

// Name of a device's group in the scene, so the camera can follow what is drawn (CameraControls).
export const deviceObjectName = (id) => `device-${id}`;

// Signed difference to[rad] - from[rad] along the shortest arc, in (-PI, PI].
const shortestAngle = (from, to) => {
  const d = (to - from) % (2 * Math.PI);
  if (d > Math.PI) return d - 2 * Math.PI;
  if (d <= -Math.PI) return d + 2 * Math.PI;
  return d;
};

// A new sample of a stream (`m`) starts its motion now, lasting the stream's average gap × the margin.
const retime = (m, now) => {
  if (m.lastSample !== null) {
    const gap = Math.min(Math.max(now - m.lastSample, MIN_SAMPLE_GAP_MS), MAX_SAMPLE_GAP_MS);
    m.gapMs = m.gapMs === null ? gap : m.gapMs + (gap - m.gapMs) * SAMPLE_GAP_SMOOTHING;
    m.duration = m.gapMs * PLAYOUT_MARGIN;
  }
  m.start = now;
  m.lastSample = now;
};

// Reusable objects to avoid per-frame allocations.
const _qYaw = new THREE.Quaternion();
const _qPitch = new THREE.Quaternion();
const _axisY = new THREE.Vector3(0, 1, 0);
const _axisX = new THREE.Vector3(1, 0, 0);

const Device = ({ id, loc, isSelected, model3d }) => {
  const meshRef = useRef();
  const camRef = useRef();
  const { invalidate } = useThree();

  // From where the drone was drawn when the last sample arrived, to that sample, over `duration` ms.
  const motionRef = useRef(null);
  if (motionRef.current === null) {
    motionRef.current = {
      from: new THREE.Vector3(),
      to: new THREE.Vector3(),
      fromYaw: 0,
      toYaw: 0,
      start: 0,
      duration: MIN_SAMPLE_GAP_MS,
      gapMs: null, // average gap between samples
      lastSample: null,
    };
  }
  // The camera's orientation in the drone (its frustum is the CameraHelper), eased the same way: applied at
  // once it jumped on every batch, and its yaw, relative to the course, jumped against the turning body.
  const gimbalRef = useRef(null);
  if (gimbalRef.current === null) {
    gimbalRef.current = {
      pitch: 0, // as drawn now
      yaw: 0,
      fromPitch: 0,
      toPitch: 0,
      fromYaw: 0,
      toYaw: 0,
      start: 0,
      duration: MIN_SAMPLE_GAP_MS,
      gapMs: null,
      lastSample: null,
    };
  }

  // What the device IS (its device model's 3D model), not the name of its category.
  const model = useGLTF(getModelPath(model3d));
  // Cloned once per model: a new clone on every render swapped the whole scene graph on each tick.
  const modelScene = useMemo(() => model.scene.clone(), [model]);

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

  // Scene axes: x east, y up, z = -north.
  const x = loc.pos[0];
  const y = loc.pos[2];
  const z = -loc.pos[1];
  const { course, gimbalPitch, gimbalYaw } = loc;

  // react-doctor/no-event-handler false positive: the sample is live telemetry
  // pushed over WebSocket/Redux, not a value this component can react to from
  // a click/submit handler. meshRef/camRef are also only attached after commit,
  // so this mutation must run in an effect, not during render.
  // Keyed on the values, not on `loc`: every tick builds new objects for all devices,
  // and a device that did not move must not restart (or re-time) its motion.
  useEffect(() => {
    const m = motionRef.current;
    const mesh = meshRef.current;
    const yaw = course !== undefined ? -(course * Math.PI) / 180 : m.toYaw;
    if (m.lastSample === null || !mesh) {
      m.from.set(x, y, z);
      m.fromYaw = yaw;
    } else {
      // Start from where it is drawn, so a sample arriving mid-motion never makes it jump.
      m.from.copy(mesh.position);
      m.fromYaw = mesh.rotation.y;
    }
    m.to.set(x, y, z);
    m.toYaw = m.fromYaw + shortestAngle(m.fromYaw, yaw);
    retime(m, performance.now());
    invalidate();
  }, [x, y, z, course, invalidate]);

  useEffect(() => {
    if (gimbalPitch === undefined) return;
    const g = gimbalRef.current;
    const pitch = (gimbalPitch * Math.PI) / 180;
    const yaw = gimbalYaw !== undefined ? (-(gimbalYaw - course) * Math.PI) / 180 : 0;
    if (g.lastSample === null) {
      g.pitch = pitch;
      g.yaw = yaw;
    }
    g.fromPitch = g.pitch;
    g.toPitch = pitch;
    g.fromYaw = g.yaw;
    g.toYaw = g.yaw + shortestAngle(g.yaw, yaw);
    retime(g, performance.now());
    invalidate();
  }, [gimbalPitch, gimbalYaw, course, invalidate]);

  useFrame(() => {
    const now = performance.now();
    const mesh = meshRef.current;
    const m = motionRef.current;
    let moving = false;
    if (mesh && m.lastSample !== null) {
      const t = Math.min((now - m.start) / m.duration, 1);
      mesh.position.lerpVectors(m.from, m.to, t);
      mesh.rotation.y = m.fromYaw + (m.toYaw - m.fromYaw) * t;
      moving = t < 1;
    }
    const g = gimbalRef.current;
    if (camRef.current && g.lastSample !== null) {
      const t = Math.min((now - g.start) / g.duration, 1);
      g.pitch = g.fromPitch + (g.toPitch - g.fromPitch) * t;
      g.yaw = g.fromYaw + (g.toYaw - g.fromYaw) * t;
      _qYaw.setFromAxisAngle(_axisY, g.yaw);
      _qPitch.setFromAxisAngle(_axisX, g.pitch);
      camRef.current.quaternion.copy(_qYaw.multiply(_qPitch));
      moving = moving || t < 1;
    }
    // Keep requesting frames until both reach their sample.
    if (moving) invalidate();
  }, DEVICE_FRAME_PRIORITY);

  if (!model) return null;

  return (
    <group ref={meshRef} name={deviceObjectName(id)}>
      <perspectiveCamera ref={camRef} fov={60} near={1} far={4} position={[0, 0.15, -0.15]} />
      <primitive object={modelScene} scale={[1, 1, 1]} />
      {/* Its lidar point cloud, only while the device page has the viewer on for this device */}
      <R3FLidarCloud deviceId={id} />
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

  const positionxyz = useMemo(() => {
    const pos = Object.values(positions).map((item) => ({
      ...item,
      lng: item.hasOwnProperty('longitude') ? item.longitude : origin3d.lng,
      lat: item.hasOwnProperty('latitude') ? item.latitude : origin3d.lat,
      alt: item.attributes?.home ? item.altitude - item.attributes.home[2] : (item.altitude ?? 0),
    }));
    const posxyz = LatLon2XYZObj(origin3d, pos, 1000);
    return posxyz.map((item) => ({
      ...item,
      name: devices[item.deviceId]?.name,
      course: positions[item.deviceId]?.course,
      gimbalPitch: positions[item.deviceId]?.attributes?.gimbal?.[0] ?? 0,
      gimbalYaw: positions[item.deviceId]?.attributes?.gimbal?.[2] ?? 0,
    }));
  }, [origin3d, positions, devices]);

  return (
    <>
      {positionxyz.map((item) => (
        <Device
          key={item.deviceId}
          id={item.deviceId}
          loc={item}
          isSelected={String(selectedDeviceId) === String(item.deviceId)}
          model3d={devices[item.deviceId]?.platform?.model3d}
        />
      ))}
    </>
  );
};

export default R3FDevices;
