import { useId, useCallback, useEffect } from 'react';
import { useSelector } from 'react-redux';
import { map } from './Mapview';
import { findFonts } from './mapUtil';
import maplibregl from 'maplibre-gl';

export const MapMissions = () => {
    const id = useId();
    const route_points = `${id}-points`;
    const clusters = `${id}-points-clusters`;
    const routes = useSelector((state) => state.mission.route);
    const mapCluster = true;
    const iconScale = 0.6;

    const createFeature = (myroute,point) => {
        return {
          id: point.id,
          name: myroute[point.routeid]['name'],
          color: '#3bb2d0',
          altitud: myroute[point.routeid]['wp'][point.id][2],
          headin: myroute[point.routeid]['wp'][point.id][3],
          attributes: myroute[point.routeid]['attributes'],
          category: 'default',
          color: 'neutral',
        };
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
            clusterMaxZoom: 17,
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
              'icon-image': 'background',
              'icon-size': iconScale,
              'icon-allow-overlap': true,
              'text-allow-overlap': true,
              'text-field': '{id}',
              'text-font': findFonts(map),
              'text-size': 14,
            },
            paint: {
              'text-halo-color': 'white',
              'text-halo-width': 1,
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
          map.on('click','mission-points',function(e) {
            new maplibregl.Popup()
            .setLngLat(e.lngLat)
            .setHTML('<h4>'+ "Ruta: "+ e.features[0].properties.name+ '</h4>' +
                    '<p>'+"Altitud:   " + e.features[0].properties.altitud + '</p>' +
                    '<p>' +"Heading:   "+ e.features[0].properties.headin + '</p>')
            .addTo(map);
          } )
    
          return () => {
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


    function routesToFeature(item){
        let waypoint_pos = Object.values(item.wp).map(function(it) {
            return [it[1], it[0]];
         });
        return {
            id: item.id,
            type: 'Feature',
            geometry: {
                type: 'LineString',
                coordinates: waypoint_pos,
                },
            properties: {
                name: item.name,
                color: '#3bb2d0',
              },
          };
    }
    function routeTowaypoints(myroute){
        let waypoint = [];
        Object.values(myroute).forEach( rt => {
            Object.keys(rt.wp).forEach( wp_key => {
                waypoint.push({longitude: rt['wp'][wp_key][1],latitude:rt['wp'][wp_key][0], id:wp_key,routeid:rt['id']});
            })
          })
        return waypoint;
    }



    useEffect(() => {
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
          features: Object.values(routes).map((route) => routesToFeature(route)),
        });
      }, [routes]);

    return null;
}
export default MapMissions;