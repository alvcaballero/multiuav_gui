// direction on line
// https://stackoverflow.com/questions/53257291/how-to-make-a-custom-line-layer-in-mapbox-gl
// example 2
// https://maplibre.org/maplibre-gl-js/docs/examples/cluster-html/
//https://docs.maptiler.com/sdk-js/examples/elevation-profile/
import { useId, useCallback, useState, useEffect, useContext } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { map } from '../MapView';
import { findFonts } from '../mapUtil';
import { missionActions } from '../../store'; // here update device action with position of uav for update in map
import palette from '../../common/palette';
import { MissionContext } from '../../components/MissionController';

class keepvalue {
  constructor() {
    this.routes = [];
    this.selecwp = { id: -1 };
  }
  initroute(routes) {
    this.routes = JSON.parse(JSON.stringify(routes));
  }
  getroute() {
    return this.routes;
  }
  setroute(routes) {
    this.routes = routes;
  }
  getSelectwp() {
    return this.selecwp;
  }
  setSelecwp(value) {
    this.selecwp = value;
  }
}

export const MapMissionsCreate = () => {
  const id = useId();
  const route_points = `${id}-points`;
  const clusters = `${id}-points-clusters`;
  const routes = useSelector((state) => state.mission.route);
  const selecwp = useSelector((state) => state.mission.selectpoint);
  const mapCluster = true;
  const iconScale = 0.6;
  const dispatch = useDispatch();
  const [testkeepvalue, settestkeepvalue] = useState(new keepvalue());
  const missionContext = useContext(MissionContext);

  let canvas = map.getCanvasContainer();

  function onMove(e) {
    let coords = e.lngLat;
    // Set a UI indicator for dragging.
    canvas.style.cursor = 'grabbing';
    // Update the Point feature in `geojson` coordinates
    // and call setData to the source layer `point` on it.
    console.log('on move point' + coords.lng + '-' + coords.lat);
    let auxroute = testkeepvalue.getroute();
    //console.log(auxroute);

    let auxselectpoint = testkeepvalue.getSelectwp();
    if (auxselectpoint.id >= 0) {
      auxroute[auxselectpoint.route_id]['wp'][auxselectpoint.id]['pos'][0] = e.lngLat.lat;
      auxroute[auxselectpoint.route_id]['wp'][auxselectpoint.id]['pos'][1] = e.lngLat.lng;
    }

    let waypoint_position = routeTowaypoints(auxroute);
    map.getSource(route_points).setData({
      type: 'FeatureCollection',
      features: waypoint_position.map((position) => ({
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [position.longitude, position.latitude],
        },
        properties: createFeature(auxroute, position),
      })),
    });

    map.getSource(id).setData({
      type: 'FeatureCollection',
      features: auxroute.map((route) => routesToFeature(route)),
    });
  }
  function onUp(e) {
    let coords = e.lngLat;

    canvas.style.cursor = '';
    let auxselectpoint = testkeepvalue.getSelectwp();
    console.log(`Longitude: ${coords.lng} Latitude: ${coords.lat}`);
    missionContext.setChangeWp({
      wp_id: auxselectpoint.id,
      route_id: auxselectpoint.route_id,
      lng: e.lngLat.lng,
      lat: e.lngLat.lat,
    });

    testkeepvalue.setSelecwp({ id: -1 });
    // Unbind mouse/touch events
    map.off('mousemove', onMove);
    map.off('touchmove', onMove);
  }

  const onMouseEnter = () => (map.getCanvas().style.cursor = 'move');
  const onMouseLeave = () => (map.getCanvas().style.cursor = '');

  const onMouseDown = (e) => {
    e.preventDefault();

    testkeepvalue.setSelecwp(e.features[0].properties);

    // when click  visualize in list
    dispatch(missionActions.selectpoint(e.features[0].properties));

    canvas.style.cursor = 'grab';
    map.on('mousemove', onMove);
    map.once('mouseup', onUp);
  };
  const onMouseTouchStart = (e) => {
    if (e.points.length !== 1) return;
    console.log('touch start');

    // Prevent the default map drag behavior.
    e.preventDefault();

    map.on('touchmove', onMove);
    map.once('touchend', onUp);
  };

  const createFeature = (myroute, point) => {
    return {
      id: point.id,
      route_id: point.routeid,
      name: myroute[point.routeid]['name'],
      uav: myroute[point.routeid]['uav'],
      latitude: myroute[point.routeid]['wp'][point.id]['pos'][0],
      longitude: myroute[point.routeid]['wp'][point.id]['pos'][1],
      altitud: myroute[point.routeid]['wp'][point.id]['pos'][2],
      yaw: myroute[point.routeid]['wp'][point.id]['yaw'],
      gimbal: myroute[point.routeid]['wp'][point.id]['gimbal'],
      actions: myroute[point.routeid]['wp'][point.id]['action'],
      attributes: myroute[point.routeid]['attributes'],
      category: 'default',
      color: myroute[point.routeid]['id'],
    };
  };

  useEffect(() => {
    console.log('render');
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
      map.on('mouseenter', 'mission-points', onMouseEnter);
      map.on('mouseleave', 'mission-points', onMouseLeave);
      map.on('mousedown', 'mission-points', onMouseDown);
      map.on('touchstart', 'mission-points', onMouseTouchStart);

      //https://stackoverflow.com/questions/72010274/stopping-map-on-listener-in-mapbox-gl-js
      //https://stackoverflow.com/questions/63036623/how-to-disable-an-event-listener-in-mapbox
      //map.off("click", "mission-points");
      return () => {
        map.off('mouseenter', 'mission-points', onMouseEnter);
        map.off('mouseleave', 'mission-points', onMouseLeave);
        map.off('mousedown', 'mission-points', onMouseDown);
        map.off('touchstart', 'mission-points', onMouseTouchStart);
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
    let waypoint_pos = Object.values(item.wp).map(function (it) {
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
    myroute.map((rt, index_rt) => {
      rt.wp.map((wp, index_wp) => {
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
    console.log('Mapmission upload mission');
    //console.log(routes);
    testkeepvalue.initroute(routes);
    let waypoint_position = routeTowaypoints(routes);

    map.getSource(route_points).setData({
      type: 'FeatureCollection',
      features: waypoint_position.map((position) => ({
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [position.longitude, position.latitude],
        },
        properties: createFeature(routes, position),
      })),
    });

    map.getSource(id).setData({
      type: 'FeatureCollection',
      features: routes.map((route) => routesToFeature(route)),
    });
  }, [routes]);

  return null;
};
export default MapMissionsCreate;
