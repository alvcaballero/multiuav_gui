import React, { useEffect, useRef,useMemo,useState } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { useSelector } from 'react-redux';
import * as THREE from 'three';
import { useModelLoader } from './ModelLoader.jsx';
import  Drone from './Drone.jsx' 
import { LatLon2XYZ , LatLon2XYZObj} from './convertion';

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

const Device = ({ id , position}) => {
  const meshRef = useRef();
  const currentPosition = useRef(new THREE.Vector3());
  const nextPosition = useRef(new THREE.Vector3());

  const { model, error } = useModelLoader("drone");
  //const position = useSelector((state) => state.session.positions[id]);

  // Actualizamos la posición en cada frame (más eficiente que recrear el mesh)
  // Encuentra la posición inicial
  useEffect(() => {
    const loc = position.find(item => item.deviceId == id);
    if (loc) {
      nextPosition.current.set(loc.pos[0], 10, loc.pos[1])
    }
  }, [position]);
  
  useEffect(() => {
    console.log("render device")
  }, []);
  
  useFrame(() => {
    if (meshRef.current) {
      currentPosition.current.lerp(nextPosition.current, 0.07  ); 
      meshRef.current.position.copy(currentPosition.current);
    }
  });
  
  if (!model) return null;
  
  return (
    <primitive
      ref={meshRef}
      object={model.scene.clone()}
      scale={[1, 1, 1]}
    />
  );
};

const R3FDevices = () => {
  const devices = useSelector((state) => state.devices.items);
  const positions = useSelector((state) => state.session.positions);
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const origin3d = useSelector((state) => state.session.scene3d.origin);
  const [positionxyz, setPositionxyz] = useState([])
  const { gl, scene } = useThree();



  const objectIds = useMemo(() => Object.keys(positions), [positions]);


  const createFeature = (devices, position, selectedPositionId) => {
    const device = devices[position.deviceId];
    let showDirection;
    switch (directionType) {
      case 'none':
        showDirection = false;
        break;
      case 'all':
        showDirection = true;
        break;
      default:
        showDirection = selectedPositionId === position.id;
        break;
    }
    let thismission = routes.find((element) => element.uav == device.name);
    let missionColor = thismission ? thismission.id : null;
    return {
      id: position.id,
      deviceId: position.deviceId,
      name: device.name,
      fixTime: formatTime(position.fixTime, 'seconds', hours12),
      category: mapIconKey(device.category),
      color: showStatus ? getStatusColor(device.status) : 'neutral',
      rotation: position.course,
      direction: showDirection,
      mission: thismission ? true : false,
      missionColor: missionColor,
    };
  };


  useEffect(() => {
    const pos = Object.values(positions).map((item)=>{return {...item,lng:item.longitude,lat:item.latitude,alt:item.altitude}})
    const posxyz = LatLon2XYZObj(origin3d,pos,1000)
    const result= posxyz.map((item,index)=>{
      return {...item,name: devices[item.deviceId].name}
    })
    setPositionxyz(result)
  }, [origin3d,positions]);

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
    if (positionxyz) {
      positionxyz.forEach((item) => {
        const pos = item.pos;
        updateDronePosition(item.deviceId, [pos[0], 20, pos[1]], [0, 0, 0], 1);
      });
    }
  }, [positionxyz]);

  useEffect(() => {
    Object.values(positions).forEach((drone) => {
      setDroneColor(drone.deviceId, selectedDeviceId === drone.deviceId ? 'blue' : 'red');
    });
  }, [selectedDeviceId]);

  return (
    <>
      {objectIds.map((id) => (
        <Device key={id} id={id} position={positionxyz}/>
      ))}
    </>
  );
};

export default R3FDevices;
