import { use, useEffect, useMemo, useRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { useSelector } from 'react-redux';
import * as THREE from 'three';
import { decodeLidarFrame, POINT_FLOATS } from '../../common/util/lidarFrame';
import { LidarCloudContext } from './LidarCloudContext';

// The most points the server sends (its highest level, server/models/lidar/pointCloud.js).
const MAX_POINTS = 12000;
// Height (the sensor's z, m) painted blue → red. A drone seldom flies higher than this over what it sees.
const LOW_Z = -20;
const HIGH_Z = 10;
// In screen pixels, not meters: the camera of this scene sits 100+ m away, where a 30 cm point is one pixel.
const POINT_SIZE_PX = 4;

const _color = new THREE.Color();

const send = (socket, type, payload) => socket.send(JSON.stringify({ type, payload }));

/**
 * What ONE device's lidar sees, drawn in the group of that device (so it moves and turns with it).
 * Asks the server for it (`lidar:subscribe`) while mounted and stops when it goes away; the points
 * never touch Redux or React state: each frame is copied into a fixed BufferGeometry from `useFrame`.
 *
 * The cloud is in the sensor's frame (x forward, y left, z up). The device's group has the model facing
 * -z with +x to its right and +y up, so a point goes to (-y, z, -x). The mount offset of the sensor on the
 * airframe is not applied (a few decimeters).
 */
const LidarCloud = ({ deviceId, level, onStatus }) => {
  const { invalidate } = useThree();
  const connected = useSelector((state) => state.session.socket);
  const frameRef = useRef(null);
  const levelRef = useRef(level);
  const onStatusRef = useRef(onStatus);
  const sentLevelRef = useRef(level);

  const geometry = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
    g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
    g.setDrawRange(0, 0);
    return g;
  }, []);
  useEffect(() => () => geometry.dispose(), [geometry]);

  useEffect(() => {
    onStatusRef.current = onStatus;
  }, [onStatus]);

  // The socket belongs to the effect: a reconnect swaps window.websocket for one with no subscription.
  useEffect(() => {
    const socket = window.websocket;
    if (!socket || socket.readyState !== WebSocket.OPEN) return undefined;
    const report = (status) => onStatusRef.current?.(status);
    let streaming = false;

    const onMessage = (event) => {
      if (typeof event.data === 'string') {
        let data;
        try {
          data = JSON.parse(event.data);
        } catch {
          return;
        }
        const status = data.lidarStatus;
        if (!status || String(status.deviceId) !== String(deviceId)) return;
        if (!status.ok) report({ state: 'rejected', reason: status.reason });
        else if (!streaming) report({ state: 'waiting' });
        return;
      }
      const frame = decodeLidarFrame(event.data);
      if (!frame || frame.deviceId !== String(deviceId)) return;
      frameRef.current = frame;
      if (!streaming) {
        streaming = true;
        report({ state: 'streaming' });
      }
      invalidate();
    };

    socket.addEventListener('message', onMessage);
    report({ state: 'waiting' });
    sentLevelRef.current = levelRef.current;
    send(socket, 'lidar:subscribe', { deviceId: Number(deviceId), level: levelRef.current });
    return () => {
      socket.removeEventListener('message', onMessage);
      if (socket.readyState === WebSocket.OPEN) {
        send(socket, 'lidar:unsubscribe', { deviceId: Number(deviceId) });
      }
      frameRef.current = null;
      geometry.setDrawRange(0, 0);
      report({ state: 'idle' });
    };
  }, [deviceId, connected, geometry, invalidate]);

  // A new level is a new subscribe on the same stream: the server retunes it without closing the topic.
  useEffect(() => {
    levelRef.current = level;
    const socket = window.websocket;
    if (level === sentLevelRef.current || !socket || socket.readyState !== WebSocket.OPEN) return;
    sentLevelRef.current = level;
    send(socket, 'lidar:subscribe', { deviceId: Number(deviceId), level });
  }, [level, deviceId]);

  useFrame(() => {
    const frame = frameRef.current;
    if (!frame) return;
    frameRef.current = null;

    const count = Math.min(frame.count, MAX_POINTS);
    const positions = geometry.attributes.position.array;
    const colors = geometry.attributes.color.array;
    const src = frame.points;
    for (let i = 0; i < count; i++) {
      const x = src[i * POINT_FLOATS];
      const y = src[i * POINT_FLOATS + 1];
      const z = src[i * POINT_FLOATS + 2];
      positions[i * 3] = -y;
      positions[i * 3 + 1] = z;
      positions[i * 3 + 2] = -x;
      const t = Math.min(1, Math.max(0, (z - LOW_Z) / (HIGH_Z - LOW_Z)));
      _color.setHSL((1 - t) * 0.66, 1, 0.5);
      colors[i * 3] = _color.r;
      colors[i * 3 + 1] = _color.g;
      colors[i * 3 + 2] = _color.b;
    }
    geometry.attributes.position.needsUpdate = true;
    geometry.attributes.color.needsUpdate = true;
    geometry.setDrawRange(0, count);
  });

  // The bounding sphere would be the one of the first frame: the cloud moves, so never cull it. No depth
  // test: what the lidar sees of the ground lies ON the scene's ground plane and would be hidden by it.
  return (
    <points geometry={geometry} frustumCulled={false} renderOrder={10}>
      <pointsMaterial
        size={POINT_SIZE_PX}
        vertexColors
        sizeAttenuation={false}
        depthTest={false}
        depthWrite={false}
      />
    </points>
  );
};

/** Nothing unless the device page turned the viewer on for THIS device. */
const R3FLidarCloud = ({ deviceId }) => {
  const lidar = use(LidarCloudContext);
  if (!lidar || String(lidar.deviceId) !== String(deviceId)) return null;
  return <LidarCloud deviceId={lidar.deviceId} level={lidar.level} onStatus={lidar.onStatus} />;
};

export default R3FLidarCloud;
