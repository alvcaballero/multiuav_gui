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
    position: {
      x: -1005.348,
      y: -273.373,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1040.348,
        y: -308.373,
        z: 0,
      },
      max_point: {
        x: -970.348,
        y: -238.373,
        z: 120,
      },
    },
  },
  {
    name: 'A2',
    type: 'windTurbine',
    position: {
      x: -1009.446,
      y: -20.234,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1044.446,
        y: -55.234,
        z: 0,
      },
      max_point: {
        x: -974.446,
        y: 14.766,
        z: 120,
      },
    },
  },
  {
    name: 'A3',
    type: 'windTurbine',
    position: {
      x: -1011.511,
      y: 178.326,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1046.511,
        y: 143.326,
        z: 0,
      },
      max_point: {
        x: -976.511,
        y: 213.326,
        z: 120,
      },
    },
  },
  {
    name: 'A4',
    type: 'windTurbine',
    position: {
      x: -1010.936,
      y: 407.945,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1045.936,
        y: 372.945,
        z: 0,
      },
      max_point: {
        x: -975.936,
        y: 442.945,
        z: 120,
      },
    },
  },
  {
    name: 'A5',
    type: 'windTurbine',
    position: {
      x: -743.808,
      y: 411.313,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -778.808,
        y: 376.313,
        z: 0,
      },
      max_point: {
        x: -708.808,
        y: 446.313,
        z: 120,
      },
    },
  },
  {
    name: 'A6',
    type: 'windTurbine',
    position: {
      x: -747.189,
      y: 182.251,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -782.189,
        y: 147.251,
        z: 0,
      },
      max_point: {
        x: -712.189,
        y: 217.251,
        z: 120,
      },
    },
  },
  {
    name: 'B1',
    type: 'windTurbine',
    position: {
      x: -1299.049,
      y: -277.456,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1334.049,
        y: -312.456,
        z: 0,
      },
      max_point: {
        x: -1264.049,
        y: -242.456,
        z: 120,
      },
    },
  },
  {
    name: 'B2',
    type: 'windTurbine',
    position: {
      x: -1296.317,
      y: -29.761,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1331.317,
        y: -64.761,
        z: 0,
      },
      max_point: {
        x: -1261.317,
        y: 5.239,
        z: 120,
      },
    },
  },
  {
    name: 'B3',
    type: 'windTurbine',
    position: {
      x: -1295.683,
      y: 179.877,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1330.683,
        y: 144.877,
        z: 0,
      },
      max_point: {
        x: -1260.683,
        y: 214.877,
        z: 120,
      },
    },
  },
  {
    name: 'B4',
    type: 'windTurbine',
    position: {
      x: -1297.683,
      y: 407.089,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1332.683,
        y: 372.089,
        z: 0,
      },
      max_point: {
        x: -1262.683,
        y: 442.089,
        z: 120,
      },
    },
  },
  {
    name: 'B5',
    type: 'windTurbine',
    position: {
      x: -1298.353,
      y: 610.053,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1333.353,
        y: 575.053,
        z: 0,
      },
      max_point: {
        x: -1263.353,
        y: 645.053,
        z: 120,
      },
    },
  },
  {
    name: 'B6',
    type: 'windTurbine',
    position: {
      x: -1019.635,
      y: 615.233,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1054.635,
        y: 580.233,
        z: 0,
      },
      max_point: {
        x: -984.635,
        y: 650.233,
        z: 120,
      },
    },
  },
  {
    name: 'C1',
    type: 'windTurbine',
    position: {
      x: -1534.473,
      y: -510.296,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1569.473,
        y: -545.296,
        z: 0,
      },
      max_point: {
        x: -1499.473,
        y: -475.296,
        z: 120,
      },
    },
  },
  {
    name: 'C2',
    type: 'windTurbine',
    position: {
      x: -1291.59,
      y: -515.089,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1326.59,
        y: -550.089,
        z: 0,
      },
      max_point: {
        x: -1256.59,
        y: -480.089,
        z: 120,
      },
    },
  },
  {
    name: 'C3',
    type: 'windTurbine',
    position: {
      x: -994.029,
      y: -508.351,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -1029.029,
        y: -543.351,
        z: 0,
      },
      max_point: {
        x: -959.029,
        y: -473.351,
        z: 120,
      },
    },
  },
  {
    name: 'C4',
    type: 'windTurbine',
    position: {
      x: -720.138,
      y: -501.613,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -755.138,
        y: -536.613,
        z: 0,
      },
      max_point: {
        x: -685.138,
        y: -466.613,
        z: 120,
      },
    },
  },
  {
    name: 'C5',
    type: 'windTurbine',
    position: {
      x: -724.616,
      y: -267.041,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -759.616,
        y: -302.041,
        z: 0,
      },
      max_point: {
        x: -689.616,
        y: -232.041,
        z: 120,
      },
    },
  },
  {
    name: 'C6',
    type: 'windTurbine',
    position: {
      x: -743.808,
      y: -26.607,
      z: 0,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=35m, height=120m',
      caution_zone: 'cylinder: radius=50m, height=130m',
      safe_zone: 'beyond 60m radius',
    },
    safe_passages: ['approach from east at 54m AGL'],
    aabb: {
      min_point: {
        x: -778.808,
        y: -61.607,
        z: 0,
      },
      max_point: {
        x: -708.808,
        y: 8.393,
        z: 120,
      },
    },
  },
  {
    name: 'Experimental_Turbine_1',
    type: 'windTurbine',
    position: {
      x: 180.231,
      y: 43.429,
      z: 20,
    },
    zones: {
      exclusion_zone: 'cylinder: radius=40m, height=108m',
      caution_zone: 'cylinder: radius=60m, height=118m',
      safe_zone: 'beyond 70m radius',
    },
    safe_passages: ['avoid base-area east sector'],
    aabb: {
      min_point: {
        x: 140.231,
        y: 3.429,
        z: 0,
      },
      max_point: {
        x: 220.231,
        y: 83.429,
        z: 128,
      },
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
  const metersPerDegreeLng = 111320 * Math.cos((origin.lat * Math.PI) / 180);

  return {
    lat: origin.lat + y / metersPerDegreeLat,
    lng: origin.lng + x / metersPerDegreeLng,
  };
};

/**
 * Extracts radius from zone description text
 * @param {string} zoneText - Zone description (e.g., "Cylinder r=40m, z=[0..108]")
 * @returns {number} Radius in meters
 */
const extractRadiusFromZone = (zoneText) => {
  const match = zoneText.match(/radius=(\d+)m/);
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

  const sourceIds = useMemo(
    () => ({
      safe: `${id}-safe-zones`,
      caution: `${id}-caution-zones`,
      exclusion: `${id}-exclusion-zones`,
      points: `${id}-obstacle-points`,
    }),
    [id]
  );

  const layerIds = useMemo(
    () => ({
      safeFill: `${id}-safe-fill`,
      cautionFill: `${id}-caution-fill`,
      exclusionFill: `${id}-exclusion-fill`,
      exclusionBorder: `${id}-exclusion-border`,
      points: `${id}-obstacle-points`,
      labels: `${id}-obstacle-labels`,
    }),
    [id]
  );

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
      const center = xyzToLatLng(obstacle.position.x, obstacle.position.y, origin);

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
  const addSourcesAndLayers = useCallback(
    (data) => {
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
    },
    [sourceIds, layerIds, colors]
  );

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
