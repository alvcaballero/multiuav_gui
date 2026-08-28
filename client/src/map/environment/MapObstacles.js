import { useId, useEffect, useCallback, useMemo, useRef } from 'react';
import { map } from '../core/mapInstance';
import { circle } from '@turf/circle';

const DEFAULT_COLORS = {
  exclusion: '#F44336',
};

/**
 * Converts a local XY offset (meters, East/North) to geographic coordinates,
 * relative to `origin`. Used to project rectangle corners (offsets from the
 * item's own center) — not a site-wide origin, just the local reference point
 * of whichever geometry is being built.
 * @param {number} x - X offset in meters (East positive)
 * @param {number} y - Y offset in meters (North positive)
 * @param {Object} origin - Reference point { lat, lng }
 * @returns {Object} { lat, lng }
 */
const xyzToLatLng = (x, y, origin) => {
  const metersPerDegreeLat = 111320;
  const metersPerDegreeLng = 111320 * Math.cos((origin.lat * Math.PI) / 180);

  return {
    lat: origin.lat + y / metersPerDegreeLat,
    lng: origin.lng + x / metersPerDegreeLng,
  };
};

/**
 * Creates circle feature using turf
 * @param {Object} center - { lat, lng }
 * @param {number} radiusMeters - Radius in meters
 * @param {Object} properties - Feature properties
 * @returns {Object} GeoJSON Feature
 */
const createCircleFeature = (center, radiusMeters, properties = {}) => {
  if (radiusMeters <= 0) return null;

  const circleFeature = circle(
    [center.lng, center.lat],
    radiusMeters / 1000, // turf uses kilometers
    { steps: 64, units: 'kilometers' },
  );

  return {
    ...circleFeature,
    properties: {
      ...circleFeature.properties,
      ...properties,
    },
  };
};

/**
 * Creates a rectangle feature (footprint) centered on `center`, rotated by
 * `yawDeg`. `widthMeters` runs along the local East-West axis and
 * `lengthMeters` along the local North-South axis at yaw=0 — same convention
 * as the `Obstacle` geometry (mcp_server/src/schemas/missions.ts) and
 * `attributes.geometry` on ElementType/ElementItem. `yawDeg` is clockwise from
 * North, matching this app's heading convention (0=North, 90=East).
 */
const createRectangleFeature = (center, widthMeters, lengthMeters, yawDeg = 0, properties = {}) => {
  if (widthMeters <= 0 || lengthMeters <= 0) return null;

  const halfWidth = widthMeters / 2;
  const halfLength = lengthMeters / 2;
  const localCorners = [
    [-halfWidth, -halfLength],
    [halfWidth, -halfLength],
    [halfWidth, halfLength],
    [-halfWidth, halfLength],
  ];

  const rad = (yawDeg * Math.PI) / 180;
  const ring = localCorners.map(([dx, dy]) => {
    const rx = dx * Math.cos(rad) + dy * Math.sin(rad);
    const ry = -dx * Math.sin(rad) + dy * Math.cos(rad);
    const { lat, lng } = xyzToLatLng(rx, ry, center);
    return [lng, lat];
  });
  ring.push(ring[0]); // GeoJSON polygons must close the ring

  return {
    type: 'Feature',
    geometry: { type: 'Polygon', coordinates: [ring] },
    properties,
  };
};

/**
 * MapObstacles Component
 * Renders inspection-element footprints on the map as a single exclusion
 * boundary each. Each obstacle: { name, type, latitude, longitude,
 * geometry_type: 'circle'|'rectangle', dimensions: {radius}|{width,length}, yaw }.
 * Positions arrive as real lat/lng (from ElementItem/ElementType), no XYZ
 * conversion or site origin involved.
 *
 * @param {Object} props
 * @param {Array} props.obstacles - Array of obstacle objects (shape above)
 * @param {boolean} props.visible - Whether to show obstacles (default: true)
 * @param {Object} props.colors - Custom colors for zones
 */
const MapObstacles = ({ obstacles = [], visible = true, colors = DEFAULT_COLORS }) => {
  const id = useId();

  const sourceIds = useMemo(
    () => ({
      exclusion: `${id}-exclusion-zones`,
    }),
    [id],
  );

  const layerIds = useMemo(
    () => ({
      exclusionFill: `${id}-exclusion-fill`,
      exclusionBorder: `${id}-exclusion-border`,
    }),
    [id],
  );

  const processObstacles = useCallback(() => {
    if (!obstacles.length) {
      return { exclusionFeatures: [] };
    }

    const exclusionFeatures = [];

    obstacles.forEach((obstacle) => {
      if (!obstacle.geometry_type || obstacle.latitude == null || obstacle.longitude == null) {
        return;
      }

      const baseProperties = {
        name: obstacle.name,
        type: obstacle.type,
        metadata: obstacle.metadata,
      };
      const center = { lat: obstacle.latitude, lng: obstacle.longitude };
      const feature =
        obstacle.geometry_type === 'circle'
          ? createCircleFeature(center, obstacle.dimensions?.radius, {
              ...baseProperties,
              zone: 'exclusion',
            })
          : createRectangleFeature(
              center,
              obstacle.dimensions?.width,
              obstacle.dimensions?.length,
              obstacle.yaw || 0,
              { ...baseProperties, zone: 'exclusion' },
            );
      if (feature) exclusionFeatures.push(feature);
    });

    return { exclusionFeatures };
  }, [obstacles]);

  // Creates the source (empty) and its layers. Data is never passed in here —
  // the data effect below owns every update via setData, so creation and
  // updates can't race each other.
  const addSourcesAndLayers = useCallback(() => {
    if (!map.getSource(sourceIds.exclusion)) {
      map.addSource(sourceIds.exclusion, {
        type: 'geojson',
        data: { type: 'FeatureCollection', features: [] },
      });
    }

    if (!map.getLayer(layerIds.exclusionFill)) {
      map.addLayer({
        id: layerIds.exclusionFill,
        type: 'fill',
        source: sourceIds.exclusion,
        metadata: { 'traccar:title': 'ObstaclesRegions' },
        paint: {
          'fill-color': colors.exclusion,
          'fill-opacity': 0.35,
        },
      });
    }

    if (!map.getLayer(layerIds.exclusionBorder)) {
      map.addLayer({
        id: layerIds.exclusionBorder,
        type: 'line',
        source: sourceIds.exclusion,
        metadata: { 'traccar:title': 'ObstaclesRegions' },
        paint: {
          'line-color': '#D32F2F',
          'line-width': 2,
          'line-dasharray': [4, 2],
        },
      });
    }
  }, [sourceIds, layerIds, colors]);

  // Function to remove all sources and layers
  const removeSourcesAndLayers = useCallback(() => {
    Object.values(layerIds).forEach((layerId) => {
      if (map.getLayer(layerId)) {
        map.removeLayer(layerId);
      }
    });

    Object.values(sourceIds).forEach((sourceId) => {
      if (map.getSource(sourceId)) {
        map.removeSource(sourceId);
      }
    });
  }, [sourceIds, layerIds]);

  // Pushes the current footprints into the existing source. Safe to call at any
  // time: a missing source (style still loading, layers not created yet) is a
  // no-op, and the layer-lifecycle effect will re-sync right after creating it.
  const syncData = useCallback(() => {
    map.getSource(sourceIds.exclusion)?.setData({
      type: 'FeatureCollection',
      features: processObstacles().exclusionFeatures,
    });
  }, [sourceIds.exclusion, processObstacles]);

  // The styledata listener below outlives any given `syncData` identity, so it
  // reads the latest one through a ref instead of closing over a stale one.
  const syncDataRef = useRef(syncData);
  syncDataRef.current = syncData;

  // Layer lifecycle: create source + layers once, tear them down on unmount or
  // when visibility/colors change. Deliberately does NOT depend on `obstacles` —
  // re-adding MapLibre layers on every geometry edit is what made yaw changes
  // (and any other dimension change) fail to repaint.
  useEffect(() => {
    if (!visible) return undefined;

    addSourcesAndLayers();
    syncDataRef.current();

    // A style reload drops custom sources/layers, so re-create and re-fill them.
    let styleDataTimeout;
    const onStyleData = () => {
      styleDataTimeout = setTimeout(() => {
        if (!map.getSource(sourceIds.exclusion)) {
          addSourcesAndLayers();
          syncDataRef.current();
        }
      }, 100);
    };

    map.on('styledata', onStyleData);

    return () => {
      map.off('styledata', onStyleData);
      clearTimeout(styleDataTimeout);
      removeSourcesAndLayers();
    };
  }, [visible, addSourcesAndLayers, removeSourcesAndLayers, sourceIds.exclusion]);

  // Data updates: the only path that repaints footprints once layers exist.
  useEffect(() => {
    if (!visible) return;
    syncData();
  }, [visible, syncData]);

  return null;
};

export default MapObstacles;
