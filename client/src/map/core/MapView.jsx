import { useRef, useLayoutEffect, useEffect, useState } from 'react';

import maplibregl from 'maplibre-gl';

import { usePreference } from '../../shared/preferences';
import usePersistedState from '../../shared/usePersistedState';

import useMapStyles from './useMapStyles';
import { map, element, switcher, addReadyListener, removeReadyListener } from './mapInstance';

const MapView = ({ children }) => {
  const containerElRef = useRef(null);

  const [mapReady, setMapReady] = useState(false);

  const mapStyles = useMapStyles();
  const activeMapStyles = 'osm,locationIqStreets,carto,googleSatellite,martin,custom';
  const [defaultMapStyle] = usePersistedState('selectedMapStyle', usePreference('map', 'osm'));
  const mapboxAccessToken = 'my tocken';
  const maxZoom = 21;

  useEffect(() => {
    if (maxZoom) {
      map.setMaxZoom(maxZoom);
    }
  }, [maxZoom]);
  useEffect(() => {
    console.log('Initializing map...');
  }, [mapReady]);

  useEffect(() => {
    maplibregl.accessToken = mapboxAccessToken;
  }, [mapboxAccessToken]);

  useEffect(() => {
    console.log('Updating map styles...');
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
    const currentEl = containerElRef.current;
    currentEl.appendChild(element);
    map.resize();
    return () => {
      currentEl.removeChild(element);
    };
  }, [containerElRef]);

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%' }} ref={containerElRef}>
      {mapReady && children}
    </div>
  );
};

export default MapView;
