import React, { useState, useEffect, Fragment } from 'react';
import { useSelector } from 'react-redux';

import * as THREE from 'three';
import maplibregl from 'maplibre-gl';
import palette from '../common/palette';
import { Line } from '@react-three/drei';

import NumberedSphere from './NumberedSphere';
import { LatLon2XYZ } from './convertion';

const R3FMission = ({ routes = [] }) => {
  const [routeLines, setRouteLines] = useState([]);
  const [routeWP, setRouteWP] = useState([]);
  const origin3d = useSelector((state) => state.session.scene3d.origin);


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

  function routesToLines(routes) {
    let routelineVector = routes.map((rt) => {
      let mylineVector3 = rt.map((point) => [point[0], point[2], point[1]])
      return mylineVector3;
    });
    return routelineVector;
  }

  function routesToXYZ(origin,routes) {
    let routesXYZ = routes.map((rt, index_rt) => {
      const position = rt.wp.map((wp, index_wp) => {return { lng: wp['pos'][1], lat: wp['pos'][0], alt: wp['pos'][2]}})
       return LatLon2XYZ(origin,position)
    })
    return routesXYZ
  }

  useEffect(() => {
    if (routes.length > 0) {
      console.log(routes)

      let origin = origin3d
      if(routes.length >0 && routes[0].wp?.pos){
        origin = {lat: routes[0].wp.pos[0] ,lng: routes[0].wp.pos[1],alt:0}
      }
      let routexyz = routesToXYZ(origin,routes)
      console.log(routexyz)
      setRouteWP(routesTowaypoints(routexyz))
      //setRouteLines(routesToLines(routexyz));
      setRouteLines(routesToLines(routexyz));
    }
  }, [routes,origin3d]);



  return (
    <Fragment>

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
    </Fragment>
  )
}
export default R3FMission