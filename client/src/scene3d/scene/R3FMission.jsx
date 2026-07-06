import { Fragment, useMemo } from 'react';
import { useSelector } from 'react-redux';

import palette from '../../shared/palette';
import { Line } from '@react-three/drei';

import NumberedSphere from '../primitives/NumberedSphere';
import { LatLon2XYZ, LatLon2XYZObj } from '../core/convertion';

const HIDE_RADIUS = 3; // meters: hide waypoint sphere when a device is within this distance

function createFeature(myroute, point) {
  let myYaw = null;
  let gimbal_pitch = null;
  if (
    myroute[point.routeid].wp[point.id].hasOwnProperty('action') &&
    myroute[point.routeid].wp[point.id].action?.hasOwnProperty('yaw')
  ) {
    myYaw = myroute[point.routeid].wp[point.id].action.yaw;
  } else if (myroute[point.routeid].wp[point.id].hasOwnProperty('yaw')) {
    myYaw = myroute[point.routeid].wp[point.id].yaw;
  }

  if (
    myroute[point.routeid].wp[point.id].hasOwnProperty('action') &&
    myroute[point.routeid].wp[point.id].action?.hasOwnProperty('gimbal')
  ) {
    gimbal_pitch = myroute[point.routeid].wp[point.id].action.gimbal;
  } else if (myroute[point.routeid].wp[point.id].hasOwnProperty('gimbal')) {
    gimbal_pitch = myroute[point.routeid].wp[point.id].gimbal;
  }

  return {
    id: point.id,
    route_id: point.routeid,
    name: myroute[point.routeid].name,
    yaw: myYaw,
    gimbal_pitch: gimbal_pitch,
    color: palette.colors_devices[point.routeid],
  };
}

function routesTowaypoints(myroute, originalRoutes) {
  const waypoint = [];
  myroute.forEach((rt, indexRt) => {
    rt.forEach((wp, indexWp) => {
      const feature = createFeature(originalRoutes, { id: indexWp, routeid: indexRt });
      waypoint.push({
        x: wp[0],
        y: wp[2],
        z: -wp[1],
        properties: feature,
      });
    });
  });
  return waypoint;
}

function routesToLines(routes) {
  let routelineVector = routes
    .map((rt) => rt.map((point) => [point[0], point[2], -point[1]]))
    .filter((line) => line.length >= 2);
  return routelineVector;
}

function routesToXYZ(origin, routes) {
  let routesXYZ = routes.map((rt) => {
    const position = rt.wp.map((wp) => {
      return { lng: wp['pos'][1], lat: wp['pos'][0], alt: wp['pos'][2] };
    });
    return LatLon2XYZ(origin, position);
  });
  return routesXYZ;
}

const R3FMission = ({ routes = [] }) => {
  const origin3d = useSelector((state) => state.session.scene3d.origin);
  const positions = useSelector((state) => state.session.positions);

  const devicePositionsXYZ = useMemo(() => {
    const pos = Object.values(positions).map((item) => ({
      ...item,
      lng: item.hasOwnProperty('longitude') ? item.longitude : origin3d.lng,
      lat: item.hasOwnProperty('latitude') ? item.latitude : origin3d.lat,
      alt: item.attributes?.home ? item.altitude - item.attributes.home[2] : (item.altitude ?? 0),
    }));
    return LatLon2XYZObj(origin3d, pos, 1000);
  }, [positions, origin3d]);

  const routesXYZ = useMemo(() => {
    return routes.length > 0 ? routesToXYZ(origin3d, routes) : [];
  }, [routes, origin3d]);

  const routeWP = useMemo(() => {
    return routesXYZ.length > 0 ? routesTowaypoints(routesXYZ, routes) : [];
  }, [routesXYZ, routes]);

  const routeLines = useMemo(() => {
    return routesXYZ.length > 0 ? routesToLines(routesXYZ) : [];
  }, [routesXYZ]);

  return (
    <Fragment>
      {/* Waypoints*/}
      {routeWP.map((wp, index) => {
        const hideLabel = devicePositionsXYZ.some((dev) => {
          const dx = dev.pos[0] - wp.x;
          const dy = dev.pos[1] - wp.y;
          const dz = dev.pos[2] - wp.z;
          return Math.sqrt(dx * dx + dy * dy + dz * dz) < HIDE_RADIUS;
        });
        return (
          <Fragment key={'wp' + index}>
            <NumberedSphere
              position={[wp.x, wp.y, wp.z]}
              properties={wp.properties}
              hideLabel={hideLabel}
            />
          </Fragment>
        );
      })}

      {routeLines.map((line, index) => (
        <Fragment key={'line' + index}>
          <Line
            points={line}
            color={palette.colors_devices[index]}
            linewidth={3}
            linecap={'round'}
            linejoin={'round'}
          />
        </Fragment>
      ))}
    </Fragment>
  );
};
export default R3FMission;
