import maplibregl from 'maplibre-gl';
import { useEffect, useState } from 'react';
import { useSelector, useStore } from 'react-redux';
import { usePreference } from '../common/preferences';
import { map } from './MapView';

const MapDefaultCamera = () => {
  const store = useStore();
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  //const positions = useSelector((state) => state.session.positions);

  const defaultLatitude = usePreference('latitude');
  const defaultLongitude = usePreference('longitude');
  const defaultZoom = usePreference('zoom', 10);

  const [initialized, setInitialized] = useState(false);

  useEffect(() => {
    if (selectedDeviceId) {
      setInitialized(true);
    } else if (!initialized) {
      if (defaultLatitude && defaultLongitude) {
        map.jumpTo({
          center: [defaultLongitude, defaultLatitude],
          zoom: defaultZoom,
        });
        setInitialized(true);
      } else {
        const positions = store.getState().session.positions;
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
          setInitialized(true);
        } else if (coordinates.length) {
          const [individual] = coordinates;
          map.jumpTo({
            center: individual,
            zoom: Math.max(map.getZoom(), 10),
          });
          setInitialized(true);
        }
      }
    }
  }, [selectedDeviceId, initialized, defaultLatitude, defaultLongitude, defaultZoom]);

  return null;
};

export default MapDefaultCamera;
