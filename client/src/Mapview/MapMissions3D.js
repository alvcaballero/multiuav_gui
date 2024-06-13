// direction on line
// https://stackoverflow.com/questions/53257291/how-to-make-a-custom-line-layer-in-mapbox-gl
// example 2
// https://maplibre.org/maplibre-gl-js/docs/examples/cluster-html/
import { useId, useCallback, useEffect, useState } from 'react';
import { useSelector } from 'react-redux';
import maplibregl from 'maplibre-gl';
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';

import { map } from './MapView';
import { findFonts } from './mapUtil';
import palette from '../common/palette';

export const MapMissions3D = () => {
  const id = useId();
  const route_points = `${id}-points`;
  const clusters = `${id}-points-clusters`;
  const routes = useSelector((state) => state.mission.route);
  const devices = useSelector((state) => state.devices.items);
  const [routeLines, setRouteLines] = useState([]);

  const modelOrigin = [-6.485616, 37.144592];
  const modelAltitude = 0;
  const modelRotate = [Math.PI / 2, 0, 0];

  //const sceneOrigin = new maplibregl.LngLat(-6.485616, 37.144592);
  const modelAsMercatorCoordinate = maplibregl.MercatorCoordinate.fromLngLat(modelOrigin, modelAltitude);
  function calculateDistanceMercatorToMeters(from, to) {
    const mercatorPerMeter = from.meterInMercatorCoordinateUnits();
    // mercator x: 0=west, 1=east
    const dEast = to.x - from.x;
    const dEastMeter = dEast / mercatorPerMeter;
    // mercator y: 0=north, 1=south
    const dNorth = to.y - from.y;
    const dNorthMeter = dNorth / mercatorPerMeter;
    return { dEastMeter, dNorthMeter };
  }
  function getOrigin() {
    let origen = null;
    routes.map((rt, index_rt) => {
      rt.wp.map((wp, index_wp) => {
        if (origen == null) {
          origen = [wp['pos'][1], wp['pos'][0]];
        }
      });
    });
    return origen;
  }

  function preparelines() {
    let line = [];
    let origen = null;
    let origen2 = null;
    let lineVector3 = [];
    let routeline = [];
    let routelineVector3 = [];

    routes.map((rt, index_rt) => {
      line = [];
      rt.wp.map((wp, index_wp) => {
        if (origen == null) {
          //origen = latLonToXYZ(wp['pos'][1], wp['pos'][0], 0);
          origen = [wp['pos'][1], wp['pos'][0], 0];
          origen2 = maplibregl.MercatorCoordinate.fromLngLat({ lng: wp['pos'][1], lat: wp['pos'][0] }, 0);
          let test2 = maplibregl.MercatorCoordinate.fromLngLat({ lng: wp['pos'][1], lat: wp['pos'][0] }, wp['pos'][2]);
          let test3 = calculateDistanceMercatorToMeters(origen2, test2);
          console.log(origen2);
          console.log(test2);
          console.log(test3);
        }
        let destino = maplibregl.MercatorCoordinate.fromLngLat({ lng: wp['pos'][1], lat: wp['pos'][0] }, wp['pos'][2]);
        let distance = calculateDistanceMercatorToMeters(origen2, destino);

        //let destino = latLonToXYZ(wp['pos'][1], wp['pos'][0], wp['pos'][2]);
        //line.push([destino[0] - origen[0], destino[1] - origen[1], destino[2] - origen[2]]);
        line.push([distance.dEastMeter, distance.dNorthMeter, wp['pos'][2] - origen[2]]);
      });
      routeline.push(line);
    });
    console.log(line);
    routeline.map((line) => {
      let mylineVector3 = [];
      line.map((point) => {
        mylineVector3.push(new THREE.Vector3(point[0], point[2], point[1]));
      });
      routelineVector3.push(new THREE.BufferGeometry().setFromPoints(mylineVector3));
    });
    line.map((point) => {
      lineVector3.push(new THREE.Vector3(point[0], point[2], point[1]));
    });
    //setRouteLines(new THREE.BufferGeometry().setFromPoints(lineVector3));
    //setRouteLines(routelineVector3);
    return routelineVector3;
  }

  // configuration of the custom layer for a 3D model per the CustomLayerInterface
  const customLayer = {
    id: '3d-model',
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

      this.map = map;
      let myrouteLines = preparelines();
      // do a for bucle to add the lines
      for (let i = 0; i < myrouteLines.length; i++) {
        let material = new THREE.LineBasicMaterial({ linewidth: 10, color: palette.colors_devices[i] });
        let myline = new THREE.Line(myrouteLines[i], material);
        this.scene.add(myline);
      }

      // use the MapLibre GL JS map canvas for three.js
      this.renderer = new THREE.WebGLRenderer({
        canvas: map.getCanvas(),
        context: gl,
        antialias: true,
      });

      this.renderer.autoClear = false;
    },
    render(gl, mercatorMatrix) {
      // `queryTerrainElevation` gives us the elevation of a point on the terrain
      // **relative to the elevation of `center`**,
      // where `center` is the point on the terrain that the middle of the camera points at.
      // If we didn't account for that offset, and the scene lay on a point on the terrain that is
      // below `center`, then the scene would appear to float in the air.
      let myorigin = getOrigin();
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

      const m = new THREE.Matrix4().fromArray(mercatorMatrix);
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
    console.log('style loaded');
    map.addLayer(customLayer);

    return () => {
      if (map.getLayer('3d-model')) {
        map.removeLayer('3d-model');
      }
    };
  }, [routes]);

  return null;
};
export default MapMissions3D;
