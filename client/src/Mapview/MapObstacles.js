import { useId, useEffect, useCallback } from 'react';
import { map } from './MapView';
import { findFonts } from './mapUtil';
import circle from '@turf/circle';

// Default origin (Global Reference)
const DEFAULT_ORIGIN = {
  lat: 41.68722260607747,
  lng: -8.847745078804891,
  alt: 0,
};

// Hardcoded wind turbine obstacles
const DEFAULT_OBSTACLES = [
  {
    name: 'B1',
    type: 'windTurbine',
    position: { x: -968.162, y: -173.843, z: 0 },
    zones: {
      exclusion_zone: 'Cylinder r=40m, z=[0..108] around turbine center (includes rotor disc + safety buffer).',
      caution_zone: 'Cylinder r=60m, z=[0..108].',
      safe_zone: 'Maintain >=70m during transit when possible; inspection allowed at ~50m.',
    },
    safe_passages: ['north', 'south', 'east', 'west'],
    aabb: {
      min_point: { x: -1008.162, y: -213.843, z: 0 },
      max_point: { x: -928.162, y: -133.843, z: 108 },
    },
    metadata: 'hub 80m; rotor diam 56m; yaw 90deg (faces east); blades stopped.',
  },
  {
    name: 'B2',
    type: 'windTurbine',
    position: { x: -968.162, y: -85.996, z: 0 },
    zones: {
      exclusion_zone: 'Cylinder r=40m, z=[0..108] around turbine center.',
      caution_zone: 'Cylinder r=60m, z=[0..108].',
      safe_zone: 'Maintain >=70m during transit when possible; inspection allowed at ~50m.',
    },
    safe_passages: ['north', 'south', 'east', 'west'],
    aabb: {
      min_point: { x: -1008.162, y: -125.996, z: 0 },
      max_point: { x: -928.162, y: -45.996, z: 108 },
    },
    metadata: 'hub 80m; rotor diam 56m; yaw 90deg (faces east); blades stopped.',
  },
  {
    name: 'B3',
    type: 'windTurbine',
    position: { x: -969.657, y: -1.128, z: 0 },
    zones: {
      exclusion_zone: 'Cylinder r=40m, z=[0..108] around turbine center.',
      caution_zone: 'Cylinder r=60m, z=[0..108].',
      safe_zone: 'Maintain >=70m during transit when possible; inspection allowed at ~50m.',
    },
    safe_passages: ['north', 'south', 'east', 'west'],
    aabb: {
      min_point: { x: -1009.657, y: -41.128, z: 0 },
      max_point: { x: -929.657, y: 38.872, z: 108 },
    },
    metadata: 'hub 80m; rotor diam 56m; yaw 90deg (faces east); blades stopped.',
  },
  {
    name: 'B4',
    type: 'windTurbine',
    position: { x: -969.657, y: 88.205, z: 0 },
    zones: {
      exclusion_zone: 'Cylinder r=40m, z=[0..108] around turbine center.',
      caution_zone: 'Cylinder r=60m, z=[0..108].',
      safe_zone: 'Maintain >=70m during transit when possible; inspection allowed at ~50m.',
    },
    safe_passages: ['north', 'south', 'east', 'west'],
    aabb: {
      min_point: { x: -1009.657, y: 48.205, z: 0 },
      max_point: { x: -929.657, y: 128.205, z: 108 },
    },
    metadata: 'hub 80m; rotor diam 56m; yaw 90deg (faces east); blades stopped.',
  },
  {
    name: 'B5',
    type: 'windTurbine',
    position: { x: -848.602, y: 88.205, z: 0 },
    zones: {
      exclusion_zone: 'Cylinder r=40m, z=[0..108] around turbine center.',
      caution_zone: 'Cylinder r=60m, z=[0..108].',
      safe_zone: 'Maintain >=70m during transit when possible; inspection allowed at ~50m.',
    },
    safe_passages: ['north', 'south', 'east', 'west'],
    aabb: {
      min_point: { x: -888.602, y: 48.205, z: 0 },
      max_point: { x: -808.602, y: 128.205, z: 108 },
    },
    metadata: 'hub 80m; rotor diam 56m; yaw 90deg (faces east); blades stopped.',
  },
  {
    name: 'B6',
    type: 'windTurbine',
    position: { x: -745.482, y: 86.716, z: 0 },
    zones: {
      exclusion_zone: 'Cylinder r=40m, z=[0..108] around turbine center.',
      caution_zone: 'Cylinder r=60m, z=[0..108].',
      safe_zone: 'Maintain >=70m during transit when possible; inspection allowed at ~50m.',
    },
    safe_passages: ['north', 'south', 'east', 'west'],
    aabb: {
      min_point: { x: -785.482, y: 46.716, z: 0 },
      max_point: { x: -705.482, y: 126.716, z: 108 },
    },
    metadata: 'hub 80m; rotor diam 56m; yaw 90deg (faces east); blades stopped.',
  },
];

/**
 * Converts local XYZ coordinates (meters) to geographic coordinates (lat/lng)
 * @param {number} x - X coordinate in meters (East positive)
 * @param {number} y - Y coordinate in meters (North positive)
 * @param {Object} origin - Origin point { lat, lng, alt }
 * @returns {Object} { lat, lng }
 */
const xyzToLatLng = (x, y, origin) => {
  const metersPerDegreeLat = 111320;
  const metersPerDegreeLng = 111320 * Math.cos(origin.lat * Math.PI / 180);

  return {
    lat: origin.lat + (y / metersPerDegreeLat),
    lng: origin.lng + (x / metersPerDegreeLng),
  };
};

/**
 * Extracts radius from zone description text
 * @param {string} zoneText - Zone description (e.g., "Cylinder r=40m, z=[0..108]")
 * @returns {number} Radius in meters
 */
const extractRadiusFromZone = (zoneText) => {
  const match = zoneText.match(/r=(\d+)m/);
  return match ? parseInt(match[1], 10) : 0;
};

/**
 * Extracts minimum distance from safe zone description
 * @param {string} zoneText - Zone description (e.g., "Maintain >=70m during transit")
 * @returns {number} Radius in meters
 */
const extractSafeRadius = (zoneText) => {
  const match = zoneText.match(/>=?(\d+)m/);
  return match ? parseInt(match[1], 10) : 0;
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
    { steps: 64, units: 'kilometers' }
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
 * MapObstacles Component
 * Renders obstacle zones (exclusion, caution, safe) on the map
 *
 * @param {Object} props
 * @param {Array} props.obstacles - Array of obstacle objects with position, zones, etc.
 * @param {Object} props.origin - Origin coordinates { lat, lng, alt } for XYZ conversion
 * @param {boolean} props.visible - Whether to show obstacles (default: true)
 * @param {Object} props.colors - Custom colors for zones
 */
const MapObstacles = ({
  obstacles = DEFAULT_OBSTACLES,
  origin = DEFAULT_ORIGIN,
  visible = true,
  colors = {
    exclusion: '#F44336',
    caution: '#FFC107',
    safe: '#4CAF50',
    point: '#1976D2',
  },
}) => {
  const id = useId();

  const sourceIds = {
    safe: `${id}-safe-zones`,
    caution: `${id}-caution-zones`,
    exclusion: `${id}-exclusion-zones`,
    points: `${id}-obstacle-points`,
  };

  const layerIds = {
    safeFill: `${id}-safe-fill`,
    cautionFill: `${id}-caution-fill`,
    exclusionFill: `${id}-exclusion-fill`,
    exclusionBorder: `${id}-exclusion-border`,
    points: `${id}-obstacle-points`,
    labels: `${id}-obstacle-labels`,
  };

  const processObstacles = useCallback(() => {
    if (!origin || !obstacles.length) {
      return {
        safeFeatures: [],
        cautionFeatures: [],
        exclusionFeatures: [],
        pointFeatures: [],
      };
    }

    const safeFeatures = [];
    const cautionFeatures = [];
    const exclusionFeatures = [];
    const pointFeatures = [];

    obstacles.forEach((obstacle) => {
      // Convert XYZ to lat/lng
      const center = xyzToLatLng(
        obstacle.position.x,
        obstacle.position.y,
        origin
      );

      // Extract radii from zone descriptions
      const exclusionRadius = extractRadiusFromZone(obstacle.zones?.exclusion_zone || '');
      const cautionRadius = extractRadiusFromZone(obstacle.zones?.caution_zone || '');
      const safeRadius = extractSafeRadius(obstacle.zones?.safe_zone || '');

      const baseProperties = {
        name: obstacle.name,
        type: obstacle.type,
        metadata: obstacle.metadata,
      };

      // Create safe zone circle
      if (safeRadius > 0) {
        const feature = createCircleFeature(center, safeRadius, {
          ...baseProperties,
          zone: 'safe',
        });
        if (feature) safeFeatures.push(feature);
      }

      // Create caution zone circle
      if (cautionRadius > 0) {
        const feature = createCircleFeature(center, cautionRadius, {
          ...baseProperties,
          zone: 'caution',
        });
        if (feature) cautionFeatures.push(feature);
      }

      // Create exclusion zone circle
      if (exclusionRadius > 0) {
        const feature = createCircleFeature(center, exclusionRadius, {
          ...baseProperties,
          zone: 'exclusion',
        });
        if (feature) exclusionFeatures.push(feature);
      }

      // Create point for obstacle center
      pointFeatures.push({
        type: 'Feature',
        geometry: {
          type: 'Point',
          coordinates: [center.lng, center.lat],
        },
        properties: baseProperties,
      });
    });

    return {
      safeFeatures,
      cautionFeatures,
      exclusionFeatures,
      pointFeatures,
    };
  }, [obstacles, origin]);

  // Function to add all sources and layers
  const addSourcesAndLayers = useCallback((data) => {
    const { safeFeatures, cautionFeatures, exclusionFeatures, pointFeatures } = data;

    // Add sources if they don't exist
    if (!map.getSource(sourceIds.safe)) {
      map.addSource(sourceIds.safe, {
        type: 'geojson',
        data: { type: 'FeatureCollection', features: safeFeatures },
      });
    }

    if (!map.getSource(sourceIds.caution)) {
      map.addSource(sourceIds.caution, {
        type: 'geojson',
        data: { type: 'FeatureCollection', features: cautionFeatures },
      });
    }

    if (!map.getSource(sourceIds.exclusion)) {
      map.addSource(sourceIds.exclusion, {
        type: 'geojson',
        data: { type: 'FeatureCollection', features: exclusionFeatures },
      });
    }

    if (!map.getSource(sourceIds.points)) {
      map.addSource(sourceIds.points, {
        type: 'geojson',
        data: { type: 'FeatureCollection', features: pointFeatures },
      });
    }

    // Add layers if they don't exist (order matters: safe -> caution -> exclusion -> points)

    // Safe zone (green, most transparent)
    if (!map.getLayer(layerIds.safeFill)) {
      map.addLayer({
        id: layerIds.safeFill,
        type: 'fill',
        source: sourceIds.safe,
        paint: {
          'fill-color': colors.safe,
          'fill-opacity': 0.15,
        },
      });
    }

    // Caution zone (yellow)
    if (!map.getLayer(layerIds.cautionFill)) {
      map.addLayer({
        id: layerIds.cautionFill,
        type: 'fill',
        source: sourceIds.caution,
        paint: {
          'fill-color': colors.caution,
          'fill-opacity': 0.25,
        },
      });
    }

    // Exclusion zone (red, most visible)
    if (!map.getLayer(layerIds.exclusionFill)) {
      map.addLayer({
        id: layerIds.exclusionFill,
        type: 'fill',
        source: sourceIds.exclusion,
        paint: {
          'fill-color': colors.exclusion,
          'fill-opacity': 0.35,
        },
      });
    }

    // Exclusion zone border (dashed red line)
    if (!map.getLayer(layerIds.exclusionBorder)) {
      map.addLayer({
        id: layerIds.exclusionBorder,
        type: 'line',
        source: sourceIds.exclusion,
        paint: {
          'line-color': '#D32F2F',
          'line-width': 2,
          'line-dasharray': [4, 2],
        },
      });
    }

    // Obstacle center points
    if (!map.getLayer(layerIds.points)) {
      map.addLayer({
        id: layerIds.points,
        type: 'circle',
        source: sourceIds.points,
        paint: {
          'circle-radius': 8,
          'circle-color': colors.point,
          'circle-stroke-width': 2,
          'circle-stroke-color': '#ffffff',
        },
      });
    }

    // Obstacle labels
    if (!map.getLayer(layerIds.labels)) {
      map.addLayer({
        id: layerIds.labels,
        type: 'symbol',
        source: sourceIds.points,
        layout: {
          'text-field': ['get', 'name'],
          'text-font': findFonts(map),
          'text-size': 12,
          'text-offset': [0, 1.5],
          'text-anchor': 'top',
        },
        paint: {
          'text-color': '#333333',
          'text-halo-color': '#ffffff',
          'text-halo-width': 1,
        },
      });
    }
  }, [sourceIds, layerIds, colors]);

  // Function to remove all sources and layers
  const removeSourcesAndLayers = useCallback(() => {
    // Remove layers
    Object.values(layerIds).forEach((layerId) => {
      if (map.getLayer(layerId)) {
        map.removeLayer(layerId);
      }
    });

    // Remove sources
    Object.values(sourceIds).forEach((sourceId) => {
      if (map.getSource(sourceId)) {
        map.removeSource(sourceId);
      }
    });
  }, [sourceIds, layerIds]);

  // Function to update or create sources and layers
  const updateMap = useCallback(() => {
    if (!visible) return;

    const data = processObstacles();

    // Check if sources exist, if not create them
    if (!map.getSource(sourceIds.safe)) {
      addSourcesAndLayers(data);
    } else {
      // Update existing sources
      map.getSource(sourceIds.safe)?.setData({
        type: 'FeatureCollection',
        features: data.safeFeatures,
      });

      map.getSource(sourceIds.caution)?.setData({
        type: 'FeatureCollection',
        features: data.cautionFeatures,
      });

      map.getSource(sourceIds.exclusion)?.setData({
        type: 'FeatureCollection',
        features: data.exclusionFeatures,
      });

      map.getSource(sourceIds.points)?.setData({
        type: 'FeatureCollection',
        features: data.pointFeatures,
      });
    }
  }, [visible, processObstacles, sourceIds, addSourcesAndLayers]);

  // Initialize and update map sources and layers
  useEffect(() => {
    updateMap();

    // Listen for style changes to re-add layers
    const onStyleData = () => {
      // Small delay to ensure map is ready after style change
      setTimeout(() => {
        if (visible && !map.getSource(sourceIds.safe)) {
          updateMap();
        }
      }, 100);
    };

    map.on('styledata', onStyleData);

    // Cleanup function
    return () => {
      map.off('styledata', onStyleData);
      removeSourcesAndLayers();
    };
  }, [visible, obstacles, origin, colors, updateMap, removeSourcesAndLayers, sourceIds.safe]);

  return null;
};

export default MapObstacles;
