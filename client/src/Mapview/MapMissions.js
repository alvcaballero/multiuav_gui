// direction on line
// https://stackoverflow.com/questions/53257291/how-to-make-a-custom-line-layer-in-mapbox-gl
// example 2
// https://maplibre.org/maplibre-gl-js/docs/examples/cluster-html/
import { useId, useEffect } from 'react';
import { useSelector } from 'react-redux';
import maplibregl from 'maplibre-gl';

import { map } from './MapView';
import { findFonts } from './mapUtil';
import palette from '../common/palette';
import { createFeature, textPopUp, routesToFeature, routeTowaypoints, cleanRoute } from './transform/mission';
export const MapMissions = ({ filteredDeviceId = -1, routes = [] }) => {
  const id = useId();
  const routePoints = `${id}-points`;
  const clusters = `${id}-points-clusters`;
  //const routes = useSelector((state) => state.mission.route);
  const devices = useSelector((state) => state.devices.items);

  const mapCluster = true;
  const iconScale = 0.6;

  const onMouseEnter = () => (map.getCanvas().style.cursor = 'pointer');
  const onMouseLeave = () => (map.getCanvas().style.cursor = '');

  const WaypointDetail = (e) => {
    const properties = JSON.parse(JSON.stringify(e.features[0].properties));
    const attributes = properties.attributes ? JSON.parse(properties.attributes) : null;
    const actions = properties.actions ? JSON.parse(properties.actions) : null;
    const html = textPopUp({properties,attributes,actions});

    new maplibregl.Popup()
      .setLngLat(e.lngLat)
      .setHTML(html)
      .addTo(map);
  };

  useEffect(() => {
    if (true) {
      map.addSource(routePoints, {
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
        source: routePoints,
        filter: ['!has', 'point_count'],
        layout: {
          'icon-image': '{category}-{color}',
          'icon-size': iconScale,
          'icon-allow-overlap': false,
          'text-allow-overlap': false,
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
        source: routePoints,
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
        if (map.getSource(routePoints)) {
          map.removeSource(routePoints);
        }
      };
    }
  }, []);


  useEffect(() => {
    if (!routes) {
      return;
    }
    const myRoutes = cleanRoute(routes);
    const routerFiltered = myRoutes.filter(
      (route) => filteredDeviceId < 0 || route.uav === devices[filteredDeviceId].name
    );
    console.log('mission filtered');
    console.log(routerFiltered);
    const waypointPosition = routeTowaypoints(routerFiltered);

    map.getSource(routePoints).setData({
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

  useEffect(() => {
    console.log('render');
  }, []);

  return null;
};
export default MapMissions;
