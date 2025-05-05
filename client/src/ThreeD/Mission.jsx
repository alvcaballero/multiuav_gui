import React, { useState, useEffect, Fragment } from 'react';
import * as THREE from 'three';
import maplibregl from 'maplibre-gl';
import palette from '../common/palette';

import NumberedSphere from '../ThreeD/NumberedSphere';

const Mission = ({routes = []}) =>{
    const [routeLines, setRouteLines] = useState([]);

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
      function routeTowaypoints(routes) {
        let waypoints = [];
        routes.map((rt, index_rt) => {
          rt.wp.map((wp, index_wp) => {
            waypoints.push(wp);
          });
        });
        return waypoints;
      }
      function routesToFeature(routes) {
        let features = [];
        routes.map((rt, index_rt) => {
          features.push(rt);
        });
        return features;
      }

      useEffect(() => {
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
        setRouteLines(routelineVector3);
      }, [routes]);

    return(
        <Fragment>
                          <NumberedSphere position={[0, 2, 0]} number={1} />
              <NumberedSphere position={[2, 2, 0]} number={2} />
              <NumberedSphere position={[-2, 2, 0]} number={3} />
        
              {React.Children.toArray(
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
          )}
        </Fragment>
    )
}
export default Mission