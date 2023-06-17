//https://maplibre.org/maplibre-gl-js-docs/example/drag-a-point/
// elevation proffile map libre
//https://documentation.maptiler.com/hc/en-us/articles/4405444518545-How-to-draw-elevation-profile-for-your-path
//custom draw
// https://medium.com/nyc-planning-digital/building-a-custom-draw-mode-for-mapbox-gl-draw-1dab71d143ee
import { useId, useCallback, useEffect } from 'react';
import { useSelector } from 'react-redux';
import { map } from '../Mapview';
import { findFonts } from '../mapUtil';
import maplibregl from 'maplibre-gl';
import { borderRadius, display, lineHeight } from '@mui/system';
import MapboxDraw from '@mapbox/mapbox-gl-draw';
import theme from './theme';


const draw = new MapboxDraw({
  displayControlsDefault: false,
  controls: {
    polygon: true,
    line_string: true,
    trash: true,
  },
  userProperties: true,
  styles: [...theme, {
    id: 'gl-draw-title',
    type: 'symbol',
    filter: ['all'],
    layout: {
      'text-field': '{user_name}',
      'text-font': ['Roboto Regular'],
      'text-size': 12,
    },
    paint: {
      'text-halo-color': 'white',
      'text-halo-width': 1,
    },
  }],
});





export const MapMissionsCreate = () => {

    let canvas = map.getCanvasContainer();
    var geojson = {
      'type': 'FeatureCollection',
      'features': [
          {
          'type': 'Feature',
          'geometry': {
            'type': 'Point',
            'coordinates': [0, 0]
                      }
          }
        ]
      };

      function onMove(e) {
        var coords = e.lngLat;
         
        // Set a UI indicator for dragging.
        canvas.style.cursor = 'grabbing';
         
        // Update the Point feature in `geojson` coordinates
        // and call setData to the source layer `point` on it.
        geojson.features[0].geometry.coordinates = [coords.lng, coords.lat];
        map.getSource('point').setData(geojson);
        }
        function onUp(e) {
          var coords = e.lngLat;
           
          // Print the coordinates of where the point had
          // finished being dragged to on the map.
          canvas.style.cursor = '';
           
          // Unbind mouse/touch events
          map.off('mousemove', onMove);
          map.off('touchmove', onMove);
        }

          useEffect(() =>{
            map.addSource('point', {
                'type': 'geojson',
                'data': geojson
              });
               
            map.addLayer({
              'id': 'point',
              'type': 'circle',
              'source': 'point',
              'paint': {
                  'circle-radius': 10,
                  'circle-color': '#3887be'
                }
            });
               
            // When the cursor enters a feature in the point layer, prepare for dragging.
            map.on('mouseenter', 'point', function () {
              map.setPaintProperty('point', 'circle-color', '#3bb2d0');
              canvas.style.cursor = 'move';
            });
              
            map.on('mouseleave', 'point', function () {
              map.setPaintProperty('point', 'circle-color', '#3887be');
              canvas.style.cursor = '';
            });
              
            map.on('mousedown', 'point', function (e) {
              // Prevent the default map drag behavior.
              e.preventDefault();
              
              canvas.style.cursor = 'grab';
              
              map.on('mousemove', onMove);
              map.once('mouseup', onUp);
            });
              
            map.on('touchstart', 'point', function (e) {
              if (e.points.length !== 1) return;
              
              // Prevent the default map drag behavior.
              e.preventDefault();
              
              map.on('touchmove', onMove);
              map.once('touchend', onUp);
            });

            map.on('click', function(e) {
              // The event object (e) contains information like the
              // coordinates of the point on the map that was clicked.
              console.log('A click event has occurred at ' + e.lngLat);
              });

            map.addControl(draw, 'top-left');


            return() =>{
              if (map.getLayer('point')) {
                map.removeLayer('point');
              }
              if (map.getSource('point')) {
                map.removeSource('point');
              }
            }

          })

    return null;
}
export default MapMissionsCreate;