import maplibregl from 'maplibre-gl';
import { useEffect, useRef, useState } from 'react';
import { useSelector } from 'react-redux';
import { useLocation } from 'react-router-dom';
import { usePreference } from '../../shared/preferences';
import { map } from '../core/MapView';

// Rutas que renderizan una vista 3D (Scene3DCanvas) en vez de MainMap/MapDefaultCamera.
const THREED_ROUTES = ['/3Dview', '/3Deditor', '/3Dmission'];
const isThreeDPath = (pathname) =>
  THREED_ROUTES.some((p) => pathname.startsWith(p)) || pathname.startsWith('/device3d');

// Viven fuera del componente: MapDefaultCamera se desmonta/remonta en cada
// navegación entre páginas 2D (Main/Mission/Planning/Device), no solo al
// volver de 3D. El mapa (singleton en MapView.jsx) nunca pierde su cámara.
// - appliedDefaultCamera evita repetir el centrado inicial (drones/preferencia)
//   en cada vuelta a 2D, pisando la posición donde el usuario dejó el mapa.
// - lastPathname permite distinguir, en cada montaje, si la página anterior
//   era una vista 3D real (donde SÍ queremos aplicar el scene3d.origin que
//   dejó el Pegman/misión) de un simple cambio entre páginas 2D (donde NO
//   queremos pisar la posición donde el usuario dejó el mapa).
let appliedDefaultCamera = false;
let lastPathname = null;

const MapDefaultCamera = () => {
  const location = useLocation();
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

  // Al volver de una vista 3D (remontaje precedido por una ruta 3D), la
  // cámara se recentra según lo que haya cambiado mientras se estuvo en 3D
  // — independiente del flag "initialized" (que solo cubre el centrado
  // inicial único de arranque por drones/preferencia). Con UAV seleccionado
  // manda su posición; si no, manda el origen 3D (Pegman/misión).
  // Corre SOLO al montar (no reacciona a selecciones/deselecciones
  // posteriores en la misma página) y solo cuando venimos de una vista 3D.
  const cameFrom3D = useRef(lastPathname !== null && isThreeDPath(lastPathname));
  useEffect(() => {
    if (!cameFrom3D.current) return;
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
  }, []);

  useEffect(() => {
    lastPathname = location.pathname;
    return () => {
      lastPathname = location.pathname;
    };
  }, [location.pathname]);

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
        new maplibregl.LngLatBounds(coordinates[0], coordinates[1]),
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
  }, [
    selectedDeviceId,
    initialized,
    defaultLatitude,
    defaultLongitude,
    defaultZoom,
    positions,
    scene3dOrigin,
  ]);

  return null;
};

export default MapDefaultCamera;
