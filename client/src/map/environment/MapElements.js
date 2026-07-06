import { useId, useEffect, useRef } from 'react';
import { useSelector } from 'react-redux';
import { map } from '../core/MapView';
import { getMapImageItems } from '../../store/sessionSelectors';

/**
 * Renders georreferenced raster images for elements that have mapImage corners defined.
 * Each item in an element group with corners [[lng,lat]x4] (SW, SE, NE, NW) is rendered
 * as a MapLibre image source overlay.
 */
const MapElements = () => {
  const id = useId();
  const imageItems = useSelector(getMapImageItems);
  const mountedKeysRef = useRef(new Set());

  useEffect(() => {
    const currentKeys = new Set(imageItems.map((i) => i.key));

    // Add new or update existing sources/layers
    imageItems.forEach(({ key, url, coordinates }) => {
      const sourceId = `${id}-img-${key}`;
      const layerId = `${id}-lyr-${key}`;

      if (!mountedKeysRef.current.has(key)) {
        map.addSource(sourceId, { type: 'image', url, coordinates });
        map.addLayer({ id: layerId, type: 'raster', source: sourceId });
        mountedKeysRef.current.add(key);
      } else {
        map.getSource(sourceId)?.updateImage({ url, coordinates });
      }
    });

    // Remove sources/layers no longer in the list
    mountedKeysRef.current.forEach((key) => {
      if (!currentKeys.has(key)) {
        const sourceId = `${id}-img-${key}`;
        const layerId = `${id}-lyr-${key}`;
        if (map.getLayer(layerId)) map.removeLayer(layerId);
        if (map.getSource(sourceId)) map.removeSource(sourceId);
        mountedKeysRef.current.delete(key);
      }
    });
  }, [imageItems]);

  useEffect(() => {
    return () => {
      mountedKeysRef.current.forEach((key) => {
        const sourceId = `${id}-img-${key}`;
        const layerId = `${id}-lyr-${key}`;
        if (map.getLayer(layerId)) map.removeLayer(layerId);
        if (map.getSource(sourceId)) map.removeSource(sourceId);
      });
      mountedKeysRef.current.clear();
    };
  }, []);

  return null;
};

export default MapElements;
