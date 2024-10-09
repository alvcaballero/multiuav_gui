import { useId, useEffect } from 'react';
import { useTheme } from '@mui/styles';
import { map } from './MapView';
import { findFonts } from './mapUtil';
//import gruaPng from '../resources/lastimages/bridge_crane_small.png';

const buildingImages = [
  {
    url: 'api/server/resources/bridge_crane.png',
    coordinates: [
      [-74.0061, 40.7127], // Bottom-left (SW)
      [-74.0059, 40.7127], // Bottom-right (SE)
      [-74.0059, 40.7129], // Top-right (NE)
      [-74.0061, 40.7129], // Top-left (NW)
    ],
  },
  {
    url: 'api/server/resources/bridge_crane.png',
    coordinates: [
      [-6.24210759087407, 36.51889490850252],
      [-6.241649280421939, 36.51894210221093],
      [-6.241419564431965, 36.517516026549686],
      [-6.241841834728916, 36.517483260273124],
    ],
  },
  // Add more buildings as needed
];

const MapMarkers = () => {
  const id = useId();

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
  }, []);

  return null;
};

export default MapMarkers;
