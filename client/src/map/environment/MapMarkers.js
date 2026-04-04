import { useId, useEffect } from 'react';
import { map } from '../core/MapView';
import { findFonts } from '../core/mapUtil';

const MapMarkers = ({ markers, showTitles }) => {
  const id = useId();
  const basesLayerId = `${id}-bases`;
  const elementsLayerId = `${id}-elements`;

  const iconScale = 0.8;

  function addSymbolLayer(layerId, sourceId) {
    if (showTitles) {
      map.addLayer({
        id: layerId,
        type: 'symbol',
        source: sourceId,
        filter: ['!has', 'point_count'],
        layout: {
          'icon-image': '{image}',
          'icon-size': iconScale,
          'icon-allow-overlap': true,
          'text-field': '{title}',
          'text-allow-overlap': true,
          'text-anchor': 'bottom',
          'text-offset': [0, -2 * iconScale],
          'text-font': findFonts(map),
          'text-size': 12,
        },
        paint: {
          'text-halo-color': 'white',
          'text-halo-width': 1,
        },
      });
    } else {
      map.addLayer({
        id: layerId,
        type: 'symbol',
        source: sourceId,
        layout: {
          'icon-image': '{image}',
          'icon-size': iconScale,
          'icon-allow-overlap': true,
        },
      });
    }
  }

  useEffect(() => {
    map.addSource(basesLayerId, {
      type: 'geojson',
      data: { type: 'FeatureCollection', features: [] },
    });
    map.addSource(elementsLayerId, {
      type: 'geojson',
      data: { type: 'FeatureCollection', features: [] },
    });

    addSymbolLayer(elementsLayerId, elementsLayerId);
    addSymbolLayer(basesLayerId, basesLayerId);

    return () => {
      [elementsLayerId, basesLayerId].forEach((layerId) => {
        if (map.getLayer(layerId)) map.removeLayer(layerId);
      });
      [elementsLayerId, basesLayerId].forEach((sourceId) => {
        if (map.getSource(sourceId)) map.removeSource(sourceId);
      });
    };
  }, [showTitles]);

  useEffect(() => {
    const bases = markers?.bases || [];
    map.getSource(basesLayerId)?.setData({
      type: 'FeatureCollection',
      features: bases.map((base, index) => ({
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [base.longitude, base.latitude],
        },
        properties: {
          image: 'base',
          title: base.name || String(index),
        },
      })),
    });

    const elements = markers?.elements || [];
    map.getSource(elementsLayerId)?.setData({
      type: 'FeatureCollection',
      features: elements.flatMap((group, groupIdx) =>
        group.items.map((item, itemIdx) => ({
          type: 'Feature',
          geometry: {
            type: 'Point',
            coordinates: [item.longitude, item.latitude],
          },
          properties: {
            image: group.type || 'default-neutral',
            title: item.name || `${groupIdx}-${itemIdx}`,
          },
        }))
      ),
    });
  }, [showTitles, markers]);

  return null;
};

export default MapMarkers;
