// direction on line
// https://stackoverflow.com/questions/53257291/how-to-make-a-custom-line-layer-in-mapbox-gl
// example 2
// https://maplibre.org/maplibre-gl-js/docs/examples/cluster-html/
import { useEffect, useMemo, useCallback } from 'react';
import { useSelector } from 'react-redux';
import maplibregl from 'maplibre-gl';
import * as THREE from 'three';

import { map } from '../core/mapInstance';
import palette from '../../shared/palette';

const modelOrigin = [-6.485616, 37.144592];
const modelRotate = [Math.PI / 2, 0, 0];
const WAYPOINT_MARKER_SIZE = 3; // meters, billboard diameter for the numbered wp marker

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

// numbered circle marker for a waypoint, billboard-style (always faces the camera)
function createWaypointMarker(number, color) {
  const size = 128;
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext('2d');

  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(size / 2, size / 2, size / 2 - 4, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = '#ffffff';
  ctx.font = `bold ${Math.round(size * 0.44)}px sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(String(number), size / 2, size / 2);

  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false }),
  );
  sprite.scale.set(WAYPOINT_MARKER_SIZE, WAYPOINT_MARKER_SIZE, 1);
  return sprite;
}

export const MapMissions3D = () => {
  const routes = useSelector((state) => state.mission.route);

  const getOrigin = useCallback(() => {
    let origen = null;
    routes.map((rt) => {
      rt.wp.map((wp) => {
        if (origen == null) {
          origen = [wp['pos'][1], wp['pos'][0]];
        }
      });
    });
    if (origen == null) {
      origen = modelOrigin;
    }
    return origen;
  }, [routes]);

  const prepareAssets = useCallback(() => {
    let origen = null;
    let origen2 = null;
    let routeLines = [];
    let routeWaypoints = [];

    routes.map((rt) => {
      let line = [];
      let waypoints = [];
      rt.wp.map((wp, index) => {
        if (origen == null) {
          origen = [wp['pos'][1], wp['pos'][0], 0];
          origen2 = maplibregl.MercatorCoordinate.fromLngLat(
            { lng: wp['pos'][1], lat: wp['pos'][0] },
            0,
          );
        }
        let destino = maplibregl.MercatorCoordinate.fromLngLat(
          { lng: wp['pos'][1], lat: wp['pos'][0] },
          wp['pos'][2],
        );
        let distance = calculateDistanceMercatorToMeters(origen2, destino);
        let point = [distance.dEastMeter, distance.dNorthMeter, wp['pos'][2] - origen[2]];

        line.push(point);
        waypoints.push({ point, number: index + 1 });
      });
      routeLines.push(line);
      routeWaypoints.push(waypoints);
    });

    let routeLineGeometry = routeLines.map((line) => {
      let points = line.map((point) => new THREE.Vector3(point[0], point[2], point[1]));
      return new THREE.BufferGeometry().setFromPoints(points);
    });

    return { lines: routeLineGeometry, waypoints: routeWaypoints };
  }, [routes]);

  // configuration of the custom layer for a 3D model per the CustomLayerInterface
  const customLayer = useMemo(
    () => ({
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
        let assets = prepareAssets();
        // do a for bucle to add the lines and their waypoint markers
        for (let i = 0; i < assets.lines.length; i++) {
          let color = palette.colors_devices[i];
          let material = new THREE.LineBasicMaterial({
            linewidth: 3,
            color,
          });
          let myline = new THREE.Line(assets.lines[i], material);
          this.scene.add(myline);

          assets.waypoints[i].forEach(({ point, number }) => {
            let marker = createWaypointMarker(number, color);
            marker.position.set(point[0], point[2], point[1]);
            this.scene.add(marker);
          });
        }

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
        let myorigin = getOrigin();
        const sceneOrigin = new maplibregl.LngLat(myorigin[0], myorigin[1]);

        const offsetFromCenterElevation = map.queryTerrainElevation(sceneOrigin) || 0;
        const sceneOriginMercator = maplibregl.MercatorCoordinate.fromLngLat(
          sceneOrigin,
          offsetFromCenterElevation,
        );

        const sceneTransform = {
          translateX: sceneOriginMercator.x,
          translateY: sceneOriginMercator.y,
          translateZ: sceneOriginMercator.z,
          rotateX: modelRotate[0],
          rotateY: modelRotate[1],
          rotateZ: modelRotate[2],
          scale: sceneOriginMercator.meterInMercatorCoordinateUnits(),
        };

        const rotationX = new THREE.Matrix4().makeRotationAxis(
          new THREE.Vector3(1, 0, 0),
          sceneTransform.rotateX,
        );
        const rotationY = new THREE.Matrix4().makeRotationAxis(
          new THREE.Vector3(0, 1, 0),
          sceneTransform.rotateY,
        );
        const rotationZ = new THREE.Matrix4().makeRotationAxis(
          new THREE.Vector3(0, 0, 1),
          sceneTransform.rotateZ,
        );

        const m = new THREE.Matrix4().fromArray(args.defaultProjectionData.mainMatrix);
        const l = new THREE.Matrix4()
          .makeTranslation(
            sceneTransform.translateX,
            sceneTransform.translateY,
            sceneTransform.translateZ,
          )
          .scale(
            new THREE.Vector3(sceneTransform.scale, -sceneTransform.scale, sceneTransform.scale),
          )
          .multiply(rotationX)
          .multiply(rotationY)
          .multiply(rotationZ);

        this.camera.projectionMatrix = m.multiply(l);
        this.renderer.resetState();
        this.renderer.render(this.scene, this.camera);
        map.triggerRepaint();
      },
    }),
    [getOrigin, prepareAssets],
  );

  useEffect(() => {
    console.log('style loaded');
    map.addLayer(customLayer);

    return () => {
      if (map.getLayer('3d-model')) {
        map.removeLayer('3d-model');
      }
    };
  }, [customLayer]);

  return null;
};
