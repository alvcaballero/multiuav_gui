import React, { useState, useEffect, Fragment } from 'react';
import * as THREE from 'three';
import maplibregl from 'maplibre-gl';
import palette from '../common/palette';
import { Line } from '@react-three/drei';

import Pose from '../ThreeD/Pose';
import NumberedSphere from '../ThreeD/NumberedSphere';

const Mission = ({ routes = [] }) => {
  const [routeLines, setRouteLines] = useState([]);
  const [routeWP, setRouteWP] = useState([]);


  const latLonToXYZ = (lat, lon, alt) => {
    const radius = 6371; // Earth radius in kilometers
    const phi = (90 - lat) * (Math.PI / 180);
    const theta = (lon + 180) * (Math.PI / 180);

    const x = -(radius + alt) * Math.sin(phi) * Math.cos(theta);
    const y = (radius + alt) * Math.cos(phi);
    const z = (radius + alt) * Math.sin(phi) * Math.sin(theta);

    return [x, y, z];
  };

  function calculateDistanceMercatorToMeters(from, to) {
    const mercatorPerMeter = from.meterInMercatorCoordinateUnits();
    // mercator x: 0=west, 1=east
    const dEast = to.x - from.x;
    const dEastMeter = dEast / mercatorPerMeter;
    // mercator y: 0=north, 1=south
    const dNorth = from.y - to.y;
    const dNorthMeter = dNorth / mercatorPerMeter;
    return { dEastMeter, dNorthMeter };
  }
  function routesTowaypoints(myroute) {
    const waypoint = [];
    myroute.forEach((rt, indexRt) => {
      rt.forEach((wp, indexWp) => {
        waypoint.push({
          x: wp[0],
          y: wp[2],
          z: wp[1],
          id: indexWp,
          routeid: indexRt,
        });
      });
    });
    return waypoint;
  }

  function routesToLines2(routes) {
    let routelineVector = routes.map((rt) => {
      let mylineVector3 = rt.map((point) => new THREE.Vector3(point[0], point[2], point[1]))
      return new THREE.BufferGeometry().setFromPoints(mylineVector3);
    });
    return routelineVector;
  }
  function routesToLines(routes) {
    let routelineVector = routes.map((rt) => {
      let mylineVector3 = rt.map((point) => [point[0], point[2], point[1]])
      return mylineVector3;
    });
    return routelineVector;
  }

  function routesToXYZ(routes) {
    let origen = null;
    let routesXYZ = [];

    routesXYZ = routes.map((rt, index_rt) => {
      return rt.wp.map((wp, index_wp) => {
        if (origen == null) {
          origen = maplibregl.MercatorCoordinate.fromLngLat({ lng: wp['pos'][1], lat: wp['pos'][0] }, 0);
        }
        let destino = maplibregl.MercatorCoordinate.fromLngLat({ lng: wp['pos'][1], lat: wp['pos'][0] }, wp['pos'][2]);
        let distance = calculateDistanceMercatorToMeters(origen, destino);

        return [distance.dEastMeter, distance.dNorthMeter, wp['pos'][2]];
      });
    });
    return routesXYZ
  }

  useEffect(() => {
    let routexyz = routesToXYZ(routes)
    setRouteWP(routesTowaypoints(routexyz))
    //setRouteLines(routesToLines(routexyz));
    setRouteLines(routesToLines(routexyz));

  }, [routes]);



  return (
    <Fragment>
      {/*  esto es un  flecha en 3d*/}
      <Pose
        x={0}
        y={0}
        theta={0}
        materialProps={{
          color: new THREE.Color(0x3287a8),
          wireframe: true,
        }}
      />
      {/* Waypoints*/}
      {React.Children.toArray(
        routeWP.map((wp, index) => (
          <Fragment key={'wp' + index}>
            <NumberedSphere position={[wp.x, wp.y, wp.z]} number={wp.id} color={palette.colors_devices[wp.routeid]} />
          </Fragment>
        ))
      )}

      {React.Children.toArray(
        routeLines.map((line, index) => (
          <Fragment key={'line' + index}>
            <Line
              points={line}
              color={palette.colors_devices[index]}
              linewidth={5}
              linecap={'round'}
              linejoin={'round'}
            />
          </Fragment>
        ))
      )}

      {/* lines*/}
      {/*React.Children.toArray(
        routeLines.map((line, index) => (
          <Fragment key={'line' + index}>
            <line geometry={line}>
              <lineBasicMaterial
                attach="material"
                color={palette.colors_devices[index]}
                linewidth={10}
                linecap={'round'}
                linejoin={'round'}
              />
            </line>
          </Fragment>
        ))
      )*/}
    </Fragment>
  )
}
export default Mission