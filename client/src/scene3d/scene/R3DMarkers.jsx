import React, { useEffect, useMemo } from 'react';
import { useThree } from '@react-three/fiber';
import { useModelLoader } from '../models/ModelLoader.jsx';
import { useSelector } from 'react-redux';
import { LatLon2XYZ } from '../core/convertion';

// heading: degrees from North, clockwise (0=N, 90=E, 180=S, 270=W)
// Three.js axes: X=East, Y=up, Z=-North → rotY = -heading_rad
const headingToRotationY = (heading = 0) => -(heading * Math.PI) / 180;

const Marker = ({ item }) => {
  const { model, error } = useModelLoader(item.type);
  const { invalidate } = useThree();

  // Clone is created inside useMemo so React owns the lifecycle — safe with Strict Mode.
  const clone = useMemo(() => {
    if (!model) return null;
    const c = model.scene.clone();
    c.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = false;
        child.receiveShadow = false;
        // Let the PBR material pick up the scene environment map.
        if (child.material) {
          const mats = Array.isArray(child.material) ? child.material : [child.material];
          mats.forEach((m) => {
            if (m.isMeshStandardMaterial) {
              m.envMapIntensity = 1.2;
              m.needsUpdate = true;
            }
          });
        }
      }
    });
    return c;
  }, [model]);

  // Dispose the clone when it's replaced or the marker unmounts.
  useEffect(() => {
    return () => {
      if (clone) {
        clone.traverse((child) => {
          if (child.isMesh) {
            child.geometry.dispose();
            if (Array.isArray(child.material)) child.material.forEach((m) => m.dispose());
            else child.material.dispose();
          }
        });
      }
    };
  }, [clone]);

  // Tell R3F to draw a frame when the model finishes loading.
  useEffect(() => {
    if (clone) invalidate();
  }, [clone, invalidate]);

  // Sync position/rotation imperatively to avoid remounting the primitive.
  useEffect(() => {
    if (!clone) return;
    clone.position.set(...item.pos);
    clone.rotation.set(0, headingToRotationY(item.heading), 0);
    invalidate();
  }, [clone, item.pos, item.heading, invalidate]);

  if (error || !clone) return null;

  return <primitive object={clone} />;
};

const R3DMarkers = ({ elements }) => {
  const origin3d = useSelector((state) => state.session.scene3d.origin);
  const range = useSelector((state) => state.session.scene3d.range);
  const { invalidate } = useThree();

  function list2Points(mylist) {
    const waypoints = [];
    if (mylist?.elements) {
      mylist.elements.forEach((conjunto, index_cj) => {
        conjunto.items.forEach((items, item_index) => {
          waypoints.push({ ...items, type: conjunto.type, title: `${index_cj}-${item_index}` });
        });
      });
    }
    if (mylist?.bases) {
      mylist.bases.forEach((items, item_index) => {
        waypoints.push({ ...items, type: 'base', title: item_index });
      });
    }
    return waypoints;
  }

  const markers = useMemo(() => {
    const listelemnts = list2Points(elements);
    const pos = listelemnts.map(({ latitude, longitude }) => ({
      lng: longitude,
      lat: latitude,
      alt: 0,
    }));
    const posxyz = LatLon2XYZ(origin3d, pos);
    const elementxyz = listelemnts.map((element, index) => ({
      ...element,
      pos: [posxyz[index][0], posxyz[index][2], -posxyz[index][1]],
    }));
    return elementxyz.filter(
      (item) =>
        item.pos[0] > -range && item.pos[0] < range && item.pos[2] > -range && item.pos[2] < range,
    );
  }, [origin3d, elements, range]);

  useEffect(() => {
    invalidate();
  }, [markers, invalidate]);

  return (
    <>
      {markers.map((item) => (
        <Marker key={`${item.type}-${item.title}`} item={item} />
      ))}
    </>
  );
};

export default R3DMarkers;
