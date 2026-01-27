import { useId, useEffect, useCallback, useMemo } from 'react';
import { map } from './MapView';
import { findFonts } from './mapUtil';
import circle from '@turf/circle';

// Default origin (Global Reference)
const DEFAULT_ORIGIN = {
  lat: 41.68722260607747,
  lng: -8.847745078804891,
  alt: 0,
};

// Default colors for zones
const DEFAULT_COLORS = {
  exclusion: '#F44336',
  caution: '#FFC107',
  safe: '#4CAF50',
  point: '#1976D2',
};

// Hardcoded wind turbine obstacles
const DEFAULT_OBSTACLES = [
  {
    name: 'A1',
    type: 'windTurbine',
    position: { x: -1006.862, y: -259.905, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -1046.862, y: -299.905, z: 0 },
      max_point: { x: -966.862, y: -219.905, z: 115 },
    },
  },
  {
    name: 'A2',
    type: 'windTurbine',
    position: { x: -1010.96, y: -6.765, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -1050.96, y: -46.765, z: 0 },
      max_point: { x: -970.96, y: 33.235, z: 115 },
    },
  },
  {
    name: 'A3',
    type: 'windTurbine',
    position: { x: -1013.025, y: 191.794, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -1053.025, y: 151.794, z: 0 },
      max_point: { x: -973.025, y: 231.794, z: 115 },
    },
  },
  {
    name: 'A4',
    type: 'windTurbine',
    position: { x: -734.116, y: 197.061, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -774.116, y: 157.061, z: 0 },
      max_point: { x: -694.116, y: 237.061, z: 115 },
    },
  },
  {
    name: 'A5',
    type: 'windTurbine',
    position: { x: -730.919, y: -1.322, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -770.919, y: -41.322, z: 0 },
      max_point: { x: -690.919, y: 38.678, z: 115 },
    },
  },
  {
    name: 'A6',
    type: 'windTurbine',
    position: { x: -729.553, y: -255.822, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -769.553, y: -295.822, z: 0 },
      max_point: { x: -689.553, y: -215.822, z: 115 },
    },
  },
  {
    name: 'B1',
    type: 'windTurbine',
    position: { x: -1300.564, y: -263.988, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -1340.564, y: -303.988, z: 0 },
      max_point: { x: -1260.564, y: -223.988, z: 115 },
    },
  },
  {
    name: 'B2',
    type: 'windTurbine',
    position: { x: -1297.832, y: -16.292, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -1337.832, y: -56.292, z: 0 },
      max_point: { x: -1257.832, y: 23.708, z: 115 },
    },
  },
  {
    name: 'B3',
    type: 'windTurbine',
    position: { x: -1297.197, y: 193.346, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -1337.197, y: 153.346, z: 0 },
      max_point: { x: -1257.197, y: 233.346, z: 115 },
    },
  },
  {
    name: 'B4',
    type: 'windTurbine',
    position: { x: -1299.198, y: 420.557, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -1339.198, y: 380.557, z: 0 },
      max_point: { x: -1259.198, y: 460.557, z: 115 },
    },
  },
  {
    name: 'B5',
    type: 'windTurbine',
    position: { x: -1023.255, y: 424.64, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -1063.255, y: 384.64, z: 0 },
      max_point: { x: -983.255, y: 464.64, z: 115 },
    },
  },
  {
    name: 'B6',
    type: 'windTurbine',
    position: { x: -732.285, y: 424.64, z: 0 },
    zones: {
      exclusion_zone: 'cylinder r=40m, z=[0,115] (rotor+buffer)',
      caution_zone: 'cylinder r=70m, z=[0,120]',
      safe_zone: 'keep >=80m laterally for transit; prefer z>=60m',
    },
    safe_passages: [
      'north: y>center_y+90 @ z 60-100',
      'south: y<center_y-90 @ z 60-100',
      'west: x<center_x-90 @ z 60-100',
      'east: x>center_x+90 @ z 60-100',
      'over: z>115',
    ],
    aabb: {
      min_point: { x: -772.285, y: 384.64, z: 0 },
      max_point: { x: -692.285, y: 464.64, z: 115 },
    },
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
  colors = DEFAULT_COLORS,
}) => {
  const id = useId();

  const sourceIds = useMemo(() => ({
    safe: `${id}-safe-zones`,
    caution: `${id}-caution-zones`,
    exclusion: `${id}-exclusion-zones`,
    points: `${id}-obstacle-points`,
  }), [id]);

  const layerIds = useMemo(() => ({
    safeFill: `${id}-safe-fill`,
    cautionFill: `${id}-caution-fill`,
    exclusionFill: `${id}-exclusion-fill`,
    exclusionBorder: `${id}-exclusion-border`,
    points: `${id}-obstacle-points`,
    labels: `${id}-obstacle-labels`,
  }), [id]);

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
      setTimeout(() => {
        if (visible && !map.getSource(sourceIds.safe)) {
          updateMap();
        }
      }, 100);
    };

    map.on('styledata', onStyleData);

    return () => {
      map.off('styledata', onStyleData);
      removeSourcesAndLayers();
    };
  }, [visible, obstacles, origin, colors, updateMap, removeSourcesAndLayers, sourceIds.safe]);

  return null;
};

export default MapObstacles;
