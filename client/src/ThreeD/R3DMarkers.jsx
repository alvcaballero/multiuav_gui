import React, { useEffect, useState } from 'react';
import { useModelLoader, modelKey } from './ModelLoader.jsx';
import { useSelector } from 'react-redux';
import { LatLon2XYZ } from './convertion';

const Marker = ({ item }) => {
  const { model, error } = useModelLoader(item.type);

  if (error) {
    console.error(error);
    return null;
  }

  if (!model) {
    return null;
  }

  return <primitive object={model.scene.clone()} position={item.pos} scale={[1, 1, 1]} />;
};

const R3DMarkers = ({ elements }) => {
  const [markers, setmarkers] = useState([]);
  const origin3d = useSelector((state) => state.session.scene3d.origin);

  function list2Points(mylist) {
    const waypoints = [];
    if (mylist?.elements) {
      mylist.elements.forEach((conjunto, index_cj) => {
        conjunto.items.forEach((items, item_index) => {
          const type = modelKey(conjunto.type);
          waypoints.push({ ...items, type, title: `${index_cj}-${item_index}` });
        });
      });
    }
    if (mylist?.bases) {
      mylist.bases.forEach((items, item_index) => {
        const type = modelKey('base');
        waypoints.push({ ...items, type, title: item_index });
      });
    }
    return waypoints;
  }

  useEffect(() => {
    const listelemnts = list2Points(elements);
    const pos = listelemnts.map(({ latitude, longitude }) => {
      return { lng: longitude, lat: latitude, alt: 0 };
    });
    const posxyz = LatLon2XYZ(origin3d, pos);
    const elementxyz = listelemnts.map((element, index) => {
      return { ...element, pos: [posxyz[index][0], posxyz[index][2], posxyz[index][1]] };
    });
    const result = elementxyz.filter(
      (item) => item.pos[0] > -1000 && item.pos[0] < 1000 && item.pos[2] > -1000 && item.pos[2] < 1000
    );
    console.log(result);
    setmarkers(result);
  }, [origin3d, elements]);

  return (
    <>
      {markers.map((item, index) => (
        <Marker key={index} item={item} />
      ))}
    </>
  );
};

export default R3DMarkers;
