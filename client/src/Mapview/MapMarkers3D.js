// direction on line
// https://stackoverflow.com/questions/53257291/how-to-make-a-custom-line-layer-in-mapbox-gl
// example 2
// https://maplibre.org/maplibre-gl-js/docs/examples/cluster-html/
import { useId, useCallback, useEffect, useState } from 'react';
import maplibregl from 'maplibre-gl';
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';

import { map } from './MapView';
import palette from '../common/palette';

export const MapMarkers3D = () => {
  const id = useId();
  const modelAltitude = 0;
  const modelRotate = [Math.PI / 2, 0, 0];

  // configuration of the custom layer for a 3D model per the CustomLayerInterface
  const customLayer = {
    id: '3d-markers',
    type: 'custom',
    renderingMode: '3d',
    onAdd(map, gl) {
      this.camera = new THREE.Camera();
      this.scene = new THREE.Scene();

      // create two three.js lights to illuminate the model
      const directionalLight = new THREE.DirectionalLight(0xffffff);
      directionalLight.position.set(0, -70, 100).normalize();
      this.scene.add(directionalLight);

      const directionalLight2 = new THREE.DirectionalLight(0xffffff);
      directionalLight2.position.set(0, 70, 100).normalize();
      this.scene.add(directionalLight2);

      const loader = new GLTFLoader();
      // '../resources/3d/windturbine.glb'
      loader.load(
        'https://maplibre.org/maplibre-gl-js/docs/assets/34M_17/34M_17.gltf',
        (gltf) => {
          this.scene.add(gltf.scene);
        },
        function (xhr) {
          console.log((xhr.loaded / xhr.total) * 100 + '% loaded');
        },
        function (error) {
          console.log('An error happened:', error);
        }
      );

      this.map = map;

      // use the MapLibre GL JS map canvas for three.js
      this.renderer = new THREE.WebGLRenderer({
        canvas: map.getCanvas(),
        context: gl,
        antialias: true,
      });

      this.renderer.autoClear = false;
    },
    render(gl, args) {
      // `queryTerrainElevation` gives us the elevation of a point on the terrain
      // **relative to the elevation of `center`**,
      // where `center` is the point on the terrain that the middle of the camera points at.
      // If we didn't account for that offset, and the scene lay on a point on the terrain that is
      // below `center`, then the scene would appear to float in the air.
      let myorigin = [-6.002290500565038, 37.41025256189089];
      const sceneOrigin = new maplibregl.LngLat(myorigin[0], myorigin[1]);

      const offsetFromCenterElevation = map.queryTerrainElevation(sceneOrigin) || 0;
      const sceneOriginMercator = maplibregl.MercatorCoordinate.fromLngLat(sceneOrigin, offsetFromCenterElevation);

      const sceneTransform = {
        translateX: sceneOriginMercator.x,
        translateY: sceneOriginMercator.y,
        translateZ: sceneOriginMercator.z,
        rotateX: modelRotate[0],
        rotateY: modelRotate[1],
        rotateZ: modelRotate[2],
        scale: sceneOriginMercator.meterInMercatorCoordinateUnits(),
      };

      const rotationX = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(1, 0, 0), sceneTransform.rotateX);
      const rotationY = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(0, 1, 0), sceneTransform.rotateY);
      const rotationZ = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(0, 0, 1), sceneTransform.rotateZ);

      const m = new THREE.Matrix4().fromArray(args.defaultProjectionData.mainMatrix);
      const l = new THREE.Matrix4()
        .makeTranslation(sceneTransform.translateX, sceneTransform.translateY, sceneTransform.translateZ)
        .scale(new THREE.Vector3(sceneTransform.scale, -sceneTransform.scale, sceneTransform.scale))
        .multiply(rotationX)
        .multiply(rotationY)
        .multiply(rotationZ);

      this.camera.projectionMatrix = m.multiply(l);
      this.renderer.resetState();
      this.renderer.render(this.scene, this.camera);
      map.triggerRepaint();
    },
  };

  useEffect(() => {
    console.log('3d object load ');
    map.addLayer(customLayer);

    return () => {
      if (map.getLayer('3d-model')) {
        map.removeLayer('3d-model');
      }
    };
  }, []);

  return null;
};
export default MapMarkers3D;
