import React, { useRef, useLayoutEffect, useEffect, useState } from 'react';
import maplibregl from 'maplibre-gl';
import 'maplibre-gl/dist/maplibre-gl.css';
import './Mapview.css';
import useMapStyles from './useMapStyles';
import { useAttributePreference, usePreference } from '../common/preferences';
import usePersistedState, { savePersistedState } from '../common/usePersistedState';
import { SwitcherControl } from './switcher';
import { mapImages } from './preloadImages';

const element = document.createElement('div');
  element.style.width = '100%';
  element.style.height = '100%';
  element.style.boxSizing = 'initial';

  export const map = new maplibregl.Map({
    container: element,
    attributionControl: false,
    center: [ -6.0025 , 37.412 ],
    zoom: 12
  });

  let ready = false;
  const readyListeners = new Set();
  
  const addReadyListener = (listener) => {
    readyListeners.add(listener);
    listener(ready);
  };
  
  const removeReadyListener = (listener) => {
    readyListeners.delete(listener);
  };
  
  const updateReadyValue = (value) => {
    ready = value;
    readyListeners.forEach((listener) => listener(value));
  };
  


  const initMap = async () => {
    if (ready) return;
    if (!map.hasImage('background')) {
      Object.entries(mapImages).forEach(([key, value]) => {
        map.addImage(key, value, {
          pixelRatio: window.devicePixelRatio,
        });
      });
    }
    updateReadyValue(true);
  };

map.addControl(new maplibregl.NavigationControl());

const switcher = new SwitcherControl(
    () => updateReadyValue(false),
    (styleId) => savePersistedState('selectedMapStyle', styleId),
    () => {
      map.once('styledata', () => {
        const waiting = () => {
          if (!map.loaded()) {
            setTimeout(waiting, 33);
          } else {
            initMap();
          }
        };
        waiting();
      });
    },
  );
  
  map.addControl(switcher);

  
  const MapView = ({ children }) => {
    const containerEl = useRef(null);
  
    const [mapReady, setMapReady] = useState(false);
  
    const mapStyles = useMapStyles();
    const activeMapStyles = 'osm';
    const [defaultMapStyle] = 'osm';
    const mapboxAccessToken = 'my tocken';
    const maxZoom = 20;
  
    useEffect(() => {
      if (maxZoom) {
        map.setMaxZoom(maxZoom);
      }
    }, [maxZoom]);
  
    useEffect(() => {
      maplibregl.accessToken = mapboxAccessToken;
    }, [mapboxAccessToken]);
  
    useEffect(() => {
      const filteredStyles = mapStyles.filter((s) => s.available && activeMapStyles.includes(s.id));
      const styles = filteredStyles.length ? filteredStyles : mapStyles.filter((s) => s.id === 'osm');
      switcher.updateStyles(styles, defaultMapStyle);
    }, [mapStyles, defaultMapStyle]);
  
    useEffect(() => {
      const listener = (ready) => setMapReady(ready);
      addReadyListener(listener);
      return () => {
        removeReadyListener(listener);
      };
    }, []);
  
    useLayoutEffect(() => {
      const currentEl = containerEl.current;
      currentEl.appendChild(element);
      map.resize();
      return () => {
        currentEl.removeChild(element);
      };
    }, [containerEl]);
  
    return (
      <div className='map-wrap' ref={containerEl}>
        {mapReady && children}
      </div>
    );
  };
  
  export default MapView;
  