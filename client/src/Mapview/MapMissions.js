// direction on line
// https://stackoverflow.com/questions/53257291/how-to-make-a-custom-line-layer-in-mapbox-gl
// example 2
// https://maplibre.org/maplibre-gl-js/docs/examples/cluster-html/
import { useId, useCallback, useEffect } from 'react';
import { useSelector } from 'react-redux';
import maplibregl from 'maplibre-gl';

import { map } from './MapView';
import { findFonts } from './mapUtil';
import palette from '../common/palette';

export const MapMissions = ({ filteredDeviceId = -1 }) => {
  const id = useId();
  const route_points = `${id}-points`;
  const clusters = `${id}-points-clusters`;
  const routes = useSelector((state) => state.mission.route);
  const devices = useSelector((state) => state.devices.items);

  const mapCluster = true;
  const iconScale = 0.6;

  const onMouseEnter = () => (map.getCanvas().style.cursor = 'pointer');
  const onMouseLeave = () => (map.getCanvas().style.cursor = '');

  const createFeature = (myroute, point) => {
    let myYaw = 0;
    let mySpeed = 0;
    if (
      myroute[point.routeid].wp[point.id].hasOwnProperty('action') &&
      myroute[point.routeid].wp[point.id].action.hasOwnProperty('yaw')
    ) {
      myYaw = myroute[point.routeid].wp[point.id].action.yaw;
    } else if (myroute[point.routeid].wp[point.id].hasOwnProperty('yaw')) {
      myYaw = myroute[point.routeid].wp[point.id].yaw;
    }
    if (myroute[point.routeid].wp[point.id].hasOwnProperty('speed')) {
      mySpeed = myroute[point.routeid].wp[point.id].speed;
    } else if (myroute[point.routeid].attributes && myroute[point.routeid].attributes.hasOwnProperty('idle_vel')) {
      mySpeed = myroute[point.routeid].attributes.idle_vel;
    }

    myYaw = Number(myYaw) ? Number(myYaw) : 0;
    mySpeed = Number(mySpeed) ? Number(mySpeed) : 0;
    const myCategory = myYaw === 0 ? 'background' : 'backgroundDirection';
    return {
      id: point.id,
      name: myroute[point.routeid].name,
      uav: myroute[point.routeid].uav,
      latitude: myroute[point.routeid].wp[point.id].pos[0],
      longitude: myroute[point.routeid].wp[point.id].pos[1],
      altitude: myroute[point.routeid].wp[point.id].pos[2],
      yaw: myroute[point.routeid].wp[point.id].yaw,
      speed: mySpeed,
      gimbal: myroute[point.routeid].wp[point.id].gimbal,
      actions: myroute[point.routeid].wp[point.id].action,
      attributes: myroute[point.routeid].attributes,
      category: myCategory,
      rotation: myYaw,
      color: myroute[point.routeid].id,
    };
  };

  const WaypointDetail = (e) => {
    const properties = JSON.parse(JSON.stringify(e.features[0].properties));
    const attribute = properties.attributes ? JSON.parse(properties.attributes) : null;
    const action = properties.actions ? JSON.parse(properties.actions) : null;

    let html = `<div style="color: #FF7A59;text-align: center" ><b>UAV: ${properties.uav}</b>
      <span><a href="https://www.google.com/maps?q=${properties.latitude},${properties.longitude}" target="_blank">
      Point_${properties.id}</a></span></div>
          <div><span>Route: ${properties.name}</span></div>
          <div style="display:inline"><span> Height: </span><span>${properties.altitude.toFixed(1)}m </span></div>`;
    html = properties.hasOwnProperty('speed')
      ? `${html} <div style="display:inline"><span>Speed: </span><span>${properties.speed} m/s </span></div>`
      : html;
    html = properties.hasOwnProperty('yaw')
      ? `${html} <div style="display:inline"><span>Yaw: </span><span>${properties.yaw}° </span></div>`
      : html;
    html = properties.hasOwnProperty('gimbal')
      ? `${html} <div style="display:inline"><span>Gimbal: </span><span>${properties.gimbal}° </span></div>`
      : html;
    let htmlAction = '<div><b> Waypoint actions: </b></div>';
    if (action) {
      htmlAction += Object.keys(action)
        .map((key) => {
          const unit = key === 'idle_vel' ? 'm/s' : '';
          return `<div style="display:inline"><span>${key}: </span><span>${action[key]} ${unit} </span></div>`;
        })
        .join('');
    }
    let htmlAttributes = '<div><b> Attributes_mission: </b></div>';
    htmlAttributes += Object.keys(attribute)
      .map((key) => {
        const unit = key === 'idle_vel' || key === 'max_vel' ? 'm/s' : '';
        return `<div style="display:inline"><span>${key}: </span><span>${attribute[key]} ${unit} </span></div>`;
      })
      .join('');
    new maplibregl.Popup()
      .setLngLat(e.lngLat)
      .setHTML(html + htmlAction + htmlAttributes)
      .addTo(map);
  };

  useEffect(() => {
    if (true) {
      map.addSource(route_points, {
        type: 'geojson',
        data: {
          type: 'FeatureCollection',
          features: [],
        },
        cluster: mapCluster,
        clusterMaxZoom: 10,
        clusterRadius: 50,
      });

      map.addSource(id, {
        type: 'geojson',
        data: {
          type: 'FeatureCollection',
          features: [],
        },
      });
      map.addLayer({
        source: id,
        id: 'mission-line',
        type: 'line',
        paint: {
          'line-color': ['get', 'color'],
          'line-width': 2,
        },
      });
      map.addLayer({
        source: id,
        id: 'mission-title',
        type: 'symbol',
        layout: {
          'text-field': '{name}',
          'text-font': findFonts(map),
          'text-size': 12,
        },
        paint: {
          'text-halo-color': 'white',
          'text-halo-width': 1,
        },
      });
      map.addLayer({
        id: 'mission-points',
        type: 'symbol',
        source: route_points,
        filter: ['!has', 'point_count'],
        layout: {
          'icon-image': '{category}-{color}',
          'icon-size': iconScale,
          'icon-allow-overlap': true,
          'text-allow-overlap': true,
          'text-field': '{id}',
          'text-font': findFonts(map),
          'text-size': 14,
          'icon-rotate': ['get', 'rotation'],
          'icon-rotation-alignment': 'map',
        },
        paint: {
          'text-color': 'white',
        },
      });

      map.addLayer({
        id: clusters,
        type: 'symbol',
        source: route_points,
        filter: ['has', 'point_count'],
        layout: {
          'icon-image': 'background',
          'icon-size': iconScale,
          'text-field': 'M',
          'text-font': findFonts(map),
          'text-size': 14,
        },
      });
      map.on('click', 'mission-points', WaypointDetail);
      map.on('mouseenter', 'mission-points', onMouseEnter);
      map.on('mouseleave', 'mission-points', onMouseLeave);

      return () => {
        map.off('click', 'mission-points', WaypointDetail);
        map.off('mouseenter', 'mission-points', onMouseEnter);
        map.off('mouseleave', 'mission-points', onMouseLeave);
        if (map.getLayer('mission-line')) {
          map.removeLayer('mission-line');
        }
        if (map.getLayer('mission-title')) {
          map.removeLayer('mission-title');
        }
        if (map.getLayer('mission-points')) {
          map.removeLayer('mission-points');
        }
        if (map.getLayer('mission-direction')) {
          map.removeLayer('mission-direction');
        }
        if (map.getLayer(clusters)) {
          map.removeLayer(clusters);
        }
        if (map.getSource(id)) {
          map.removeSource(id);
        }
        if (map.getSource(route_points)) {
          map.removeSource(route_points);
        }
      };
    }
    return () => {};
  }, []);

  function routesToFeature(item) {
    let waypoint_pos = item.wp.map(function (it) {
      return [it['pos'][1], it['pos'][0]];
    });
    return {
      id: item.id,
      type: 'Feature',
      geometry: {
        type: 'LineString',
        coordinates: waypoint_pos,
      },
      properties: {
        name: item.uav, //name,
        color: palette.colors_devices[item.id],
      },
    };
  }
  function routeTowaypoints(myroute) {
    const waypoint = [];
    myroute.forEach((rt, indexRt) => {
      rt.wp.forEach((wp, indexWp) => {
        waypoint.push({
          longitude: wp.pos[1],
          latitude: wp.pos[0],
          id: indexWp,
          routeid: indexRt,
        });
      });
    });
    return waypoint;
  }

  useEffect(() => {
    const routerFiltered = routes.filter(
      (route) => filteredDeviceId < 0 || route.uav === devices[filteredDeviceId].name
    );
    console.log('mission filtered');
    console.log(routerFiltered);
    const waypointPosition = routeTowaypoints(routerFiltered);

    map.getSource(route_points).setData({
      type: 'FeatureCollection',
      features: waypointPosition.map((position) => ({
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [position.longitude, position.latitude],
        },
        properties: createFeature(routerFiltered, position),
      })),
    });

    map.getSource(id).setData({
      type: 'FeatureCollection',
      features: routerFiltered.map((route) => routesToFeature(route)),
    });
  }, [routes]);

  return null;
};
export default MapMissions;
