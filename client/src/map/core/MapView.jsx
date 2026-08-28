import { useRef, useLayoutEffect, useEffect, useState, useMemo } from 'react';

import maplibregl from 'maplibre-gl';

import { usePreference } from '../../shared/preferences';
import usePersistedState from '../../shared/usePersistedState';

import useMapStyles from './useMapStyles';
import {
  map,
  element,
  initMap,
  addReadyListener,
  removeReadyListener,
  updateReadyValue,
} from './mapInstance';
import MapSwitcher from '../controls/MapSwitcher';

const MapView = ({ children }) => {
  const containerElRef = useRef(null);

  const [mapReady, setMapReady] = useState(false);

  const mapStyles = useMapStyles();
  const activeMapStyles = 'osm,locationIqStreets,carto,googleSatellite,openFreeMap,martin,custom';
  const [selectedStyleId, setSelectedStyleId] = usePersistedState(
    'selectedMapStyle',
    usePreference('map', 'locationIqStreets'),
  );
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

  const styles = useMemo(() => {
    const filtered = mapStyles.filter((s) => s.available && activeMapStyles.includes(s.id));
    return filtered.length ? filtered : mapStyles.filter((s) => s.id === 'osm');
  }, [mapStyles, activeMapStyles]);

  useEffect(() => {
    const style = styles.find((s) => s.id === selectedStyleId);
    if (!style) {
      setSelectedStyleId(styles[0].id);
      return;
    }
    updateReadyValue(false);
    map.coordinateSystem = style.coordinateSystem;
    map.setStyle(style.style, { diff: false });
    map.setTransformRequest(style.transformRequest);
    let timeoutId;
    const waiting = () => {
      if (!map.loaded()) {
        timeoutId = setTimeout(waiting, 33);
      } else {
        initMap();
        updateReadyValue(true);
      }
    };
    map.once('styledata', waiting);
    return () => clearTimeout(timeoutId);
  }, [styles, selectedStyleId, setSelectedStyleId]);

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
      <MapSwitcher styles={styles} selectedId={selectedStyleId} onSelect={setSelectedStyleId} />
      {mapReady && children}
    </div>
  );
};

export default MapView;
