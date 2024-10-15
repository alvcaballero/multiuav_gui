import { useId, useEffect } from 'react';
import { map } from './MapView';
import { useAttributePreference } from '../common/preferences';

const MapMarkers = () => {
  const id = useId();
  const buildingImages = useAttributePreference('customElements', []);

  useEffect(() => {
    buildingImages.forEach((building, index) => {
      const { url, coordinates } = building;

      // Add an image source for each building
      map.addSource(`${id}-BImage-${index}`, {
        type: 'image',
        url,
        coordinates, // Array of coordinates [SW, SE, NE, NW]
      });

      // Add a layer for each building image
      map.addLayer({
        id: `${id}-BILayer-${index}`,
        type: 'raster',
        source: `${id}-BImage-${index}`,
      });
    });

    return () => {
      buildingImages.forEach((building, index) => {
        if (map.getLayer(`${id}-BILayer-${index}`)) {
          map.removeLayer(`${id}-BILayer-${index}`);
        }
        if (map.getSource(`${id}-BImage-${index}`)) {
          map.removeSource(`${id}-BImage-${index}`);
        }
      });
    };
  }, [buildingImages]);

  return null;
};

export default MapMarkers;
