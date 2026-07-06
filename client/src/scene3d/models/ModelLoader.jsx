import { useEffect, useState } from 'react';
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';

// Definir la ruta base para los modelos
const BASE_PATH = window.location.origin;

const modelPaths = {
  windTurbine: `${BASE_PATH}/models/wind_turbine.glb`,
  base1: `${BASE_PATH}/models/base.glb`,
  base: `${BASE_PATH}/models/LandingPad.glb`,
  drone: `${BASE_PATH}/models/Drone.glb`,
  drone2: `${BASE_PATH}/models/DroneLVL2A.glb`,
  m300: `${BASE_PATH}/models/M300.glb`,
  default: `${BASE_PATH}/models/M300.glb`,
};

/**
 * Registers 3D model paths from the custom element type catalog.
 * Call once on app init (after preloadImages).
 */
const loadCustomModelPaths = async () => {
  try {
    const res = await fetch('/api/markers/types');
    if (!res.ok) return;
    const types = await res.json();
    types
      .filter((t) => t.model3d)
      .forEach((t) => {
        modelPaths[t.id] = t.model3d;
      });
  } catch {
    // server unavailable — skip
  }
};

// Cache para modelos ya cargados
const modelCache = new Map();
const loadingQueue = new Map();

const geometry = new THREE.BoxGeometry(2, 2, 2);
const material = new THREE.MeshStandardMaterial({ color: 0xff0000 }); // Color inicial
const defaultMesh = new THREE.Mesh(geometry, material);
const group = new THREE.Group();
group.add(defaultMesh);
modelCache.set('default', { scene: group });

export const modelKey = (category) => {
  switch (category) {
    case 'dji_M210_noetic':
    case 'dji_M210_melodic_rtk':
    case 'dji_M210_melodic':
    case 'dji_M210_noetic_rtk':
    case 'dji_M600':
      return 'drone';
    case 'dji_M300':
    case 'dji_M300_rtk':
      return 'm300';
    default:
      return modelPaths.hasOwnProperty(category) ? category : 'default';
  }
};

export const getModelPath = (category) => {
  return modelPaths[modelKey(category)];
};

const getModel = async (type) => {
  if (!type) return;

  // If path not registered yet, try fetching from catalog
  if (!modelPaths[type]) {
    try {
      await loadCustomModelPaths();
    } catch {
      // server unavailable — skip
    }
  }

  // Fall back to default if type has no 3D model
  const resolvedType = modelPaths[type] ? type : 'default';

  if (modelCache.has(resolvedType)) {
    return modelCache.get(resolvedType);
  }
  if (loadingQueue.has(resolvedType)) return await loadingQueue.get(resolvedType);

  const loader = new GLTFLoader();
  const modelPath = modelPaths[resolvedType];

  const loadPromise = new Promise((resolve, reject) => {
    loader.load(
      modelPath,
      (gltf) => {
        gltf.scene.traverse((child) => {
          if (child.isMesh) {
            child.castShadow = true;
            child.receiveShadow = true;
          }
        });
        modelCache.set(resolvedType, gltf);
        loadingQueue.delete(resolvedType);
        resolve(gltf);
      },
      undefined,
      (error) => {
        console.error(`Error cargando modelo "${resolvedType}":`, error);
        loadingQueue.delete(resolvedType);
        reject(error);
      },
    );
  });
  loadingQueue.set(resolvedType, loadPromise);
  return await loadPromise;
};

export const useModelLoader = (type) => {
  //const { getModel } = useContext(ModelContext);
  const [model, setModel] = useState(null);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (!type) return;
    const load = async () => {
      try {
        const gltf = await getModel(type);
        setModel(gltf);
      } catch (err) {
        setError(err);
      }
    };
    load();
  }, [type]);

  return { model, error };
};

export default useModelLoader;
