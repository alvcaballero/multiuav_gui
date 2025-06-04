import React, { useEffect, useRef,useMemo } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { useSelector } from 'react-redux';
import * as THREE from 'three';
import { useModelLoader } from './ModelLoader.jsx';
import  Drone from './Drone.jsx' 

// Objeto para almacenar las referencias a los objetos 3D de los drones
const droneObjects = {};

const createDroneObject = (id) => {
  console.log('----createDroneObject' + id);
  const geometry = new THREE.BoxGeometry(1, 1, 1);
  const material = new THREE.MeshStandardMaterial({ color: 0xff0000 }); // Color inicial
  const droneMesh = new THREE.Mesh(geometry, material);
  // droneObjects[id] = model.scene.clone();
  droneObjects[id] = droneMesh;
  return droneMesh;
};

const updateDronePosition = (id, position, rotation, scale) => {
  if (droneObjects[id]) {
    //console.log('----updateDronePosition' + id);
    droneObjects[id].position.set(...position);
    droneObjects[id].rotation.set(...rotation);
    droneObjects[id].scale.setScalar(scale);
  }
};

const setDroneColor = (id, color) => {
  if (droneObjects[id] && droneObjects[id].material) {
    droneObjects[id].material.color.set(color);
  }
};

// const R3FDevices = ({ positions, onClick, showStatus, selectedPosition, titleField }) => {

const Device = ({ id }) => {
  const meshRef = useRef();
  const { model, error } = useModelLoader("drone");
  const position = useSelector((state) => state.session.positions[id]);

  // Actualizamos la posición en cada frame (más eficiente que recrear el mesh)
  useEffect(() => {
    //console.log(position);
  }, [position]);

  if(model){  
  return (
    <primitive
      ref={meshRef}
      object={model.scene.clone()}
      position={[10,1,1]}
      scale={[1, 1, 1]} // Ajusta la escala según necesites
    />
  );}else{
    return null
  }
};

const R3FDevices = () => {
  const devices = useSelector((state) => state.devices.items);
  const positions = useSelector((state) => state.session.positions);
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const { gl, scene } = useThree();



  const objectIds = useMemo(() => Object.keys(positions), [positions]);


  useEffect(() => {
    //console.log('----R3FDevices');
  }, []);

  useEffect(() => {
    // Crear los objetos de los drones y añadirlos a la escena
    Object.values(positions).forEach((drone) => {
      const droneMesh = createDroneObject(drone.deviceId);
      scene.add(droneMesh);
    });

    return () => {
      // Limpieza: remover los objetos de la escena al desmontar (opcional)
      Object.values(positions).forEach((drone) => {
        if (droneObjects[drone.deviceId]) {
          scene.remove(droneObjects[drone.deviceId]);
          delete droneObjects[drone.deviceId];
        }
      });
    };
  }, [scene, Object.values(positions).length]);

  useEffect(() => {
    if (positions) {
      Object.keys(positions).forEach((droneId) => {
        const positionData = positions[droneId];
        updateDronePosition(droneId, [10, 10, 10], [0, 0, 0], 1);
      });
    }
  }, [positions]);

  useEffect(() => {
    Object.values(positions).forEach((drone) => {
      setDroneColor(drone.deviceId, selectedDeviceId === drone.deviceId ? 'blue' : 'red');
    });
  }, [selectedDeviceId]);

  return (
    <>
      {objectIds.map((id) => (
        <Device key={id} id={id} />
      ))}
    </>
  );
};

export default R3FDevices;
