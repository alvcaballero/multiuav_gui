import { useEffect, useState, useMemo } from 'react';
import { useLoader } from '@react-three/fiber';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import { KTX2Loader } from 'three/examples/jsm/loaders/KTX2Loader';
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader';
import * as THREE from 'three';

// Definir la ruta base para los modelos
const BASE_PATH = window.location.origin;

const modelPaths = {
    windturbine: `${BASE_PATH}/models/windturbine.gltf`,
    base: `${BASE_PATH}/models/Astronaut.glb`,
};

// Cache para modelos ya cargados
const modelCache = new Map();

export const useModelLoader = (type) => {
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
                    console.log(`Progreso de carga: ${progress.toFixed(2)}%`);
                };

                // Cargar el modelo con el manejador de progreso
                const gltf = await new Promise((resolve, reject) => {
                    loader.load(
                        modelPath,
                        resolve,
                        onProgress,
                        (error) => {
                            console.error('Error detallado al cargar el modelo:', error);
                            reject(error);
                        }
                    );
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
                console.log('Modelo cargado exitosamente');
            } catch (err) {
                console.error('Error completo:', err);
                setError(`Error al cargar el modelo: ${err.message}`);
                setLoadingProgress(0);
            }
        };

        loadModel();
    }, [type, loader]);

    return { model, error, loadingProgress };
};

export default useModelLoader; 