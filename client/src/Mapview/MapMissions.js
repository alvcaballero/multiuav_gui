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

export const MapMissions = ({ filtereddeviceid = -1 }) => {
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
    return {
      id: point.id,
      name: myroute[point.routeid]['name'],
      uav: myroute[point.routeid]['uav'],
      latitude: myroute[point.routeid]['wp'][point.id]['pos'][0],
      longitude: myroute[point.routeid]['wp'][point.id]['pos'][1],
      altitud: myroute[point.routeid]['wp'][point.id]['pos'][2],
      yaw: myroute[point.routeid]['wp'][point.id]['yaw'],
      speed: myroute[point.routeid]['wp'][point.id]['speed'],
      gimbal: myroute[point.routeid]['wp'][point.id]['gimbal'],
      actions: myroute[point.routeid]['wp'][point.id]['action'],
      attributes: myroute[point.routeid]['attributes'],
      category: 'default',
      color: myroute[point.routeid]['id'],
    };
  };

  const WaypointDetail = (e) => {
    console.log('target');
    console.log(e);
    let html = `<div style="color: #FF7A59;text-align: center" ><b>UAV: ${
      e.features[0].properties.uav
    }</b>
      <span><a href="https://www.google.com/maps?q=${e.features[0].properties.latitude},${
      e.features[0].properties.longitude
    }" target="_blank">
      Point_${e.features[0].properties.id}</a></span></div>
          <div><span>Route: ${e.features[0].properties.name}</span></div>
          <div style="display:inline"><span> Height: </span><span>${e.features[0].properties.altitud.toFixed(
            1
          )}m </span></div>`;
    html = e.features[0].properties.hasOwnProperty('speed')
      ? html +
        `<div style="display:inline"><span>Speed: </span><span>${e.features[0].properties.speed} m/s </span></div>`
      : html;
    html = e.features[0].properties.hasOwnProperty('yaw')
      ? html +
        `<div style="display:inline"><span>Yaw: </span><span>${e.features[0].properties.yaw}° </span></div>`
      : html;
    html = e.features[0].properties.hasOwnProperty('gimbal')
      ? html +
        `<div style="display:inline"><span>Gimbal: </span><span>${e.features[0].properties.gimbal}° </span></div>`
      : html;
    let html_action = '<div><b> Waypoint actions: </b></div>';
    if (e.features[0].properties.actions) {
      let action = JSON.parse(e.features[0].properties.actions);
      html_action =
        html_action +
        Object.keys(action)
          .map((key) => {
            let unit = key == 'idle_vel' ? 'm/s' : '';
            return `<div style="display:inline"><span>${key}: </span><span>${action[key]} ${unit} </span></div>`;
          })
          .join('');
    }
    let html_atributes = '<div><b> Attributes_mission: </b></div>';
    let attribute = JSON.parse(e.features[0].properties.attributes);
    html_atributes =
      html_atributes +
      Object.keys(attribute)
        .map((key) => {
          let unit = key == 'idle_vel' || key == 'max_vel' ? 'm/s' : '';
          return `<div style="display:inline"><span>${key}: </span><span>${attribute[key]} ${unit} </span></div>`;
        })
        .join('');
    new maplibregl.Popup()
      .setLngLat(e.lngLat)
      .setHTML(html + html_action + html_atributes)
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
          'icon-image': 'background-{color}',
          'icon-size': iconScale,
          'icon-allow-overlap': true,
          'text-allow-overlap': true,
          'text-field': '{id}',
          'text-font': findFonts(map),
          'text-size': 14,
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
    let waypoint = [];
    myroute.forEach((rt, index_rt) => {
      rt.wp.forEach((wp, index_wp) => {
        waypoint.push({
          longitude: wp['pos'][1],
          latitude: wp['pos'][0],
          id: index_wp,
          routeid: index_rt,
        });
      });
    });
    return waypoint;
  }

  useEffect(() => {
    let routerfiltered = routes.filter(
      (route) => filtereddeviceid < 0 || route.uav == devices[filtereddeviceid].name
    );
    console.log('mission filtered');
    console.log(routerfiltered);
    let waypointPosition = routeTowaypoints(routerfiltered);

    map.getSource(route_points).setData({
      type: 'FeatureCollection',
      features: waypointPosition.map((position) => ({
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [position.longitude, position.latitude],
        },
        properties: createFeature(routerfiltered, position),
      })),
    });

    map.getSource(id).setData({
      type: 'FeatureCollection',
      features: routerfiltered.map((route) => routesToFeature(route)),
    });
  }, [routes]);

  return null;
};
export default MapMissions;
