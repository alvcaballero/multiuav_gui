import maplibregl from 'maplibre-gl';
import { useEffect, useRef, useState } from 'react';
import { useSelector } from 'react-redux';
import { usePreference } from '../../shared/preferences';
import { map } from '../core/MapView';

// Viven fuera del componente: MapDefaultCamera se desmonta/remonta al ir y volver
// de la vista 3D, y el mapa (singleton en MapView.jsx) nunca pierde su cámara.
// - appliedDefaultCamera evita repetir el centrado inicial (drones/preferencia)
//   en cada vuelta a 2D, pisando la posición donde el usuario dejó el mapa.
// - hasMountedOnce distingue "primer montaje de toda la sesión" (donde NO
//   queremos que el scene3d.origin por defecto tape el centrado inicial) de
//   "remontaje al volver de 3D" (donde SÍ queremos aplicar el origen que dejó
//   el Pegman/misión).
let appliedDefaultCamera = false;
let hasMountedOnce = false;

const MapDefaultCamera = () => {
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const positions = useSelector((state) => state.session.positions);
  const scene3dOrigin = useSelector((state) => state.session.scene3d.origin);

  const defaultLatitude = usePreference('latitude');
  const defaultLongitude = usePreference('longitude');
  const defaultZoom = usePreference('zoom', 10);

  const [initialized, setInitialized] = useState(appliedDefaultCamera);
  const markInitialized = () => {
    appliedDefaultCamera = true;
    setInitialized(true);
  };

  // Al volver de 3D (remontaje), la cámara se recentra según lo que haya
  // cambiado mientras se estuvo en 3D — independiente del flag "initialized"
  // (que solo cubre el centrado inicial único de arranque por drones/
  // preferencia). Con UAV seleccionado manda su posición; si no, manda el
  // origen 3D (Pegman/misión).
  const isRemount = useRef(hasMountedOnce);
  useEffect(() => {
    hasMountedOnce = true;
    if (!isRemount.current) return;
    if (selectedDeviceId) {
      const position = positions[selectedDeviceId];
      if (position) {
        map.jumpTo({
          center: [position.longitude, position.latitude],
          zoom: Math.max(defaultZoom > 0 ? defaultZoom : map.getZoom(), 10),
        });
      }
      return;
    }
    if (!scene3dOrigin) return;
    map.jumpTo({
      center: [scene3dOrigin.lng, scene3dOrigin.lat],
      zoom: defaultZoom,
    });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [scene3dOrigin, selectedDeviceId]);

  useEffect(() => {
    if (initialized) return;
    if (selectedDeviceId) {
      const position = positions[selectedDeviceId];
      if (position) {
        map.jumpTo({
          center: [position.longitude, position.latitude],
          zoom: Math.max(defaultZoom > 0 ? defaultZoom : map.getZoom(), 10),
        });
        markInitialized();
        return;
      }
    }
    if (defaultLatitude && defaultLongitude) {
      map.jumpTo({
        center: [defaultLongitude, defaultLatitude],
        zoom: defaultZoom,
      });
      markInitialized();
      return;
    }
    const coordinates = Object.values(positions).map((item) => [item.longitude, item.latitude]);
    if (coordinates.length > 1) {
      const bounds = coordinates.reduce(
        (bounds, item) => bounds.extend(item),
        new maplibregl.LngLatBounds(coordinates[0], coordinates[1])
      );
      const canvas = map.getCanvas();
      map.fitBounds(bounds, {
        duration: 0,
        padding: Math.min(canvas.width, canvas.height) * 0.1,
      });
      markInitialized();
    } else if (coordinates.length) {
      const [individual] = coordinates;
      map.jumpTo({
        center: individual,
        zoom: Math.max(map.getZoom(), 10),
      });
      markInitialized();
    }
  }, [selectedDeviceId, initialized, defaultLatitude, defaultLongitude, defaultZoom, positions, scene3dOrigin]);

  return null;
};

export default MapDefaultCamera;
