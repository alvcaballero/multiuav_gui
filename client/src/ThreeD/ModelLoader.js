import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import * as THREE from 'three';

// Definir la ruta base para los modelos
const BASE_PATH = window.location.origin;

const modelPaths = {
  windturbine: `${BASE_PATH}/models/windturbine.gltf`,
  base: `${BASE_PATH}/models/Astronaut.glb`,
  drone: `${BASE_PATH}/models/Drone.glb`,
  drone2: `${BASE_PATH}/models/DroneLVL2A.glb`,
  default: `${BASE_PATH}/models/Astronaut.glb`,
};

// Cache para modelos ya cargados
const modelCache = new Map();

class ModelLoader {
  constructor() {
    this.loader = new GLTFLoader();
    this.currentModel = null;
    this.error = null;
    this.loadingProgress = 0;
    this.onProgressCallback = null;
    this.onLoadCallback = null;
    this.onErrorCallback = null;
  }

  setProgressCallback(callback) {
    this.onProgressCallback = callback;
  }

  setLoadCallback(callback) {
    this.onLoadCallback = callback;
  }

  setErrorCallback(callback) {
    this.onErrorCallback = callback;
  }

  async loadModel(type) {
    if (!type || !modelPaths[type]) {
      const error = `Modelo no encontrado para el tipo: ${type}`;
      this.error = error;
      if (this.onErrorCallback) this.onErrorCallback(error);
      return;
    }

    // Verificar si el modelo está en caché
    if (modelCache.has(type)) {
      this.currentModel = modelCache.get(type);
      if (this.onLoadCallback) this.onLoadCallback(this.currentModel);
      return;
    }

    try {
      this.loadingProgress = 0;
      if (this.onProgressCallback) this.onProgressCallback(0);

      const modelPath = modelPaths[type];
      console.log(`Intentando cargar modelo desde: ${modelPath}`);

      // Configurar el manejador de progreso
      const onProgress = (event) => {
        const progress = (event.loaded / event.total) * 100;
        this.loadingProgress = progress;
        if (this.onProgressCallback) this.onProgressCallback(progress);
        console.log(`Progreso de carga: ${progress.toFixed(2)}%`);
      };

      // Cargar el modelo con el manejador de progreso
      const gltf = await new Promise((resolve, reject) => {
        this.loader.load(modelPath, resolve, onProgress, (error) => {
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
      this.currentModel = gltf;
      this.loadingProgress = 100;
      if (this.onProgressCallback) this.onProgressCallback(100);
      if (this.onLoadCallback) this.onLoadCallback(gltf);
      console.log('Modelo cargado exitosamente');
    } catch (err) {
      console.error('Error completo:', err);
      this.error = `Error al cargar el modelo: ${err.message}`;
      this.loadingProgress = 0;
      if (this.onErrorCallback) this.onErrorCallback(this.error);
    }
  }

  getCurrentModel() {
    return this.currentModel;
  }

  getError() {
    return this.error;
  }

  getLoadingProgress() {
    return this.loadingProgress;
  }
}

export default ModelLoader;
