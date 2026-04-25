import React, { useEffect, useRef, useState } from 'react';
import { useModelLoader } from '../models/ModelLoader.jsx';
import { useSelector } from 'react-redux';
import { LatLon2XYZ } from '../core/convertion';

// heading: degrees from North, clockwise (0=N, 90=E, 180=S, 270=W)
// Three.js axes: X=East, Y=up, Z=-North → rotY = -heading_rad
const headingToRotationY = (heading = 0) => -(heading * Math.PI) / 180;

const Marker = ({ item }) => {
  const { model, error } = useModelLoader(item.type);
  const cloneRef = useRef(null);

  useEffect(() => {
    return () => {
      if (cloneRef.current) {
        cloneRef.current.traverse((child) => {
          if (child.isMesh) {
            child.geometry.dispose();
            if (Array.isArray(child.material)) {
              child.material.forEach((m) => m.dispose());
            } else {
              child.material.dispose();
            }
          }
        });
        cloneRef.current = null;
      }
    };
  }, []);

  // Sync position/rotation when item changes without remounting
  useEffect(() => {
    if (!cloneRef.current) return;
    cloneRef.current.position.set(...item.pos);
    cloneRef.current.rotation.set(0, headingToRotationY(item.heading), 0);
  }, [item.pos, item.heading]);

  if (error || !model) return null;

  if (!cloneRef.current) {
    cloneRef.current = model.scene.clone();
    cloneRef.current.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    cloneRef.current.position.set(...item.pos);
    cloneRef.current.rotation.set(0, headingToRotationY(item.heading), 0);
  }

  return (
    <primitive
      object={cloneRef.current}
      scale={[1, 1, 1]}
    />
  );
};

const R3DMarkers = ({ elements }) => {
  const [markers, setmarkers] = useState([]);
  const origin3d = useSelector((state) => state.session.scene3d.origin);
  const range = useSelector((state) => state.session.scene3d.range);

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

  useEffect(() => {
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
    const result = elementxyz.filter(
      (item) => item.pos[0] > -range && item.pos[0] < range && item.pos[2] > -range && item.pos[2] < range
    );
    setmarkers(result);
  }, [origin3d, elements, range]);

  return (
    <>
      {markers.map((item) => (
        <Marker key={`${item.type}-${item.title}`} item={item} />
      ))}
    </>
  );
};

export default R3DMarkers;
