import { useEffect, useState, useMemo } from 'react';
import { useLoader } from '@react-three/fiber';
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
export const loadCustomModelPaths = async () => {
  try {
    const res = await fetch('/api/markers/types');
    if (!res.ok) return;
    const types = await res.json();
    types
      .filter((t) => t.model3d)
      .forEach((t) => {
        modelPaths[t.id] = t.model3d;
      });
  } catch (_) {
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

export const getModel = async (type) => {
  if (!type) return;

  // If path not registered yet, try fetching from catalog
  if (!modelPaths[type]) {
    try {
      await loadCustomModelPaths();
    } catch (_) {}
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
      }
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
  }, [type, getModel]);

  return { model, error };
};

export const useModelLoader2 = (type) => {
  const [model, setModel] = useState(null);
  const [error, setError] = useState(null);
  const [loadingProgress, setLoadingProgress] = useState(0);

  const loader = useMemo(() => {
    const gltfLoader = new GLTFLoader();
    return gltfLoader;
  }, []);

  useEffect(() => {
    if (!type || !modelPaths[type]) {
      setError(`Modelo no encontrado para el tipo: ${type}`);
      return;
    }

    // Verificar si el modelo está en caché
    if (modelCache.has(type)) {
      setModel(modelCache.get(type));
      return;
    }

    const loadModel = async () => {
      try {
        setLoadingProgress(0);

        const modelPath = modelPaths[type];
        console.log(`Intentando cargar modelo desde: ${modelPath}`);

        // Configurar el manejador de progreso
        const onProgress = (event) => {
          const progress = (event.loaded / event.total) * 100;
          setLoadingProgress(progress);
          console.log(`Progreso de carga ${type} : ${progress.toFixed(2)}%`);
        };

        // Cargar el modelo con el manejador de progreso
        const gltf = await new Promise((resolve, reject) => {
          loader.load(modelPath, resolve, onProgress, (error) => {
            console.error('Error detallado al cargar el modelo:', error);
            reject(error);
          });
        });

        // Validar el modelo cargado
        if (!gltf || !gltf.scene) {
          throw new Error('Modelo inválido o corrupto');
        }

        // Optimizar el modelo
        gltf.scene.traverse((child) => {
          if (child.isMesh) {
            child.castShadow = true;
            child.receiveShadow = true;
          }
        });

        // Guardar en caché
        modelCache.set(type, gltf);
        setModel(gltf);
        setLoadingProgress(100);
        console.log('Modelo cargado exitosamente' + type);
      } catch (err) {
        console.error('Error completo:', err);
        setError(`Error al cargar el modelo:${type} ${err.message}`);
        setLoadingProgress(0);
      }
    };

    loadModel();
  }, [type, loader]);

  return { model, error, loadingProgress };
};

export default useModelLoader;
