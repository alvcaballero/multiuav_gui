import { chatLogger } from '../../common/logger.js';

// Constants for geodetic calculations
const EARTH_RADIUS_M = 6378137.0; // WGS84 equatorial radius in meters
const DEG_TO_RAD = Math.PI / 180;
const ECCENTRICITY_SQ = 0.00669438; // WGS84 first eccentricity squared

/**
 * Calculate the local origin based on device positions
 * Uses the centroid of all device positions as the origin
 * @param {Array} devices - Array of devices with location {lat, lng, alt}
 * @returns {Object} Origin {lat, lng, alt}
 */
function calculateLocalOrigin(devices) {
  if (!devices || devices.length === 0) {
    throw new Error('No devices provided to calculate local origin');
  }

  let sumLat = 0;
  let sumLng = 0;
  let sumAlt = 0;

  for (const device of devices) {
    sumLat += device.location.lat;
    sumLng += device.location.lng;
    sumAlt += 0;
  }

  return {
    lat: sumLat / devices.length,
    lng: sumLng / devices.length,
    alt: sumAlt / devices.length,
  };
}

/**
 * Convert geodetic coordinates (lat, lng) to local ENU coordinates (x, y, z)
 * X+ = East, Y+ = North, Z+ = Up
 * @param {number} lat - Latitude in degrees
 * @param {number} lng - Longitude in degrees
 * @param {number} alt - Altitude in meters
 * @param {Object} origin - Local origin {lat, lng, alt}
 * @returns {Object} Local coordinates {x, y, z}
 */
function geodeticToENU(lat, lng, alt, origin) {
  const latRad = lat * DEG_TO_RAD;
  const lngRad = lng * DEG_TO_RAD;
  const originLatRad = origin.lat * DEG_TO_RAD;
  const originLngRad = origin.lng * DEG_TO_RAD;

  // Calculate differences
  const dLat = latRad - originLatRad;
  const dLng = lngRad - originLngRad;
  const dAlt = (alt || 0) - (origin.alt || 0);

  // Local radius of curvature (prime vertical)
  const sinOriginLat = Math.sin(originLatRad);
  const rN = EARTH_RADIUS_M / Math.sqrt(1 - ECCENTRICITY_SQ * sinOriginLat * sinOriginLat);

  // Meridional radius of curvature
  const rM =
    (EARTH_RADIUS_M * (1 - ECCENTRICITY_SQ)) / Math.pow(1 - ECCENTRICITY_SQ * sinOriginLat * sinOriginLat, 1.5);

  // ENU coordinates: X+ = East, Y+ = North, Z+ = Up
  const x = dLng * (rN + (origin.alt || 0)) * Math.cos(originLatRad); // East
  const y = dLat * (rM + (origin.alt || 0)); // North
  const z = dAlt; // Up

  return { x, y, z };
}

/**
 * Convert local ENU coordinates (x, y, z) to geodetic coordinates (lat, lng, alt)
 * X+ = East, Y+ = North, Z+ = Up
 * @param {number} x - East coordinate in meters
 * @param {number} y - North coordinate in meters
 * @param {number} z - Up coordinate in meters
 * @param {Object} origin - Local origin {lat, lng, alt}
 * @returns {Object} Geodetic coordinates {lat, lng, alt}
 */
function ENUToGeodetic(x, y, z, origin) {
  const originLatRad = origin.lat * DEG_TO_RAD;

  // Local radius of curvature (prime vertical)
  const sinOriginLat = Math.sin(originLatRad);
  const rN = EARTH_RADIUS_M / Math.sqrt(1 - ECCENTRICITY_SQ * sinOriginLat * sinOriginLat);

  // Meridional radius of curvature
  const rM =
    (EARTH_RADIUS_M * (1 - ECCENTRICITY_SQ)) / Math.pow(1 - ECCENTRICITY_SQ * sinOriginLat * sinOriginLat, 1.5);

  // Convert ENU to geodetic differences
  const dLng = x / ((rN + (origin.alt || 0)) * Math.cos(originLatRad));
  const dLat = y / (rM + (origin.alt || 0));
  const dAlt = z;

  // Convert to degrees and add to origin
  const lat = origin.lat + dLat / DEG_TO_RAD;
  const lng = origin.lng + dLng / DEG_TO_RAD;
  const alt = (origin.alt || 0) + dAlt;

  return { lat, lng, alt };
}

/**
 * Convert mission data from local XYZ (ENU) coordinates to geodetic coordinates (lat, lng, alt)
 * Input structure follows MissionSchemaXYZ
 * @param {Object} missionData - Mission with XYZ coordinates following MissionSchemaXYZ structure
 * @returns {Object} Mission with geodetic coordinates (lat, lng, alt)
 */
function convertMissionXYZToLatLong(missionData) {
  if (!missionData) {
    throw new Error('Mission data is required');
  }

  if (!missionData.origin_global) {
    throw new Error('Mission data must contain origin_global for coordinate conversion');
  }

  const origin = missionData.origin_global;
  chatLogger.info(`Using origin for conversion: lat=${origin.lat}, lng=${origin.lng}, alt=${origin.alt}`);

  // Deep clone to avoid mutating original
  const converted = JSON.parse(JSON.stringify(missionData));

  // Remove origin_global from converted mission (not needed in lat/lng format)
  delete converted.origin_global;

  // Convert waypoints in each route from XYZ to lat/lng/alt
  if (converted.route && Array.isArray(converted.route)) {
    for (const route of converted.route) {
      if (route.wp && Array.isArray(route.wp)) {
        for (const waypoint of route.wp) {
          if (waypoint.pos && Array.isArray(waypoint.pos) && waypoint.pos.length === 3) {
            // pos is [x, y, z] in XYZ format
            const [x, y, z] = waypoint.pos;
            const geodetic = ENUToGeodetic(x, y, z, origin);

            // Convert to [lat, lng, alt] format
            waypoint.pos = [
              Number(geodetic.lat.toFixed(8)),
              Number(geodetic.lng.toFixed(8)),
              Number(geodetic.alt.toFixed(2)),
            ];
          }
        }
      }
    }
  }

  chatLogger.info(`Mission converted from XYZ to lat/lng coordinates. Routes: ${converted.route?.length || 0}`);
  return converted;
}

/**
 * Convert mission briefing data from geodetic coordinates to local XYZ (ENU) coordinates
 * Input structure follows filteredMissionSchema
 * @param {Object} missionBriefing - Mission briefing with filteredMissionSchema structure
 * @returns {Object} Mission briefing with XYZ coordinates and origin_global
 */
function convertMissionBriefingToXYZ(missionBriefing) {
  if (!missionBriefing) {
    throw new Error('Mission briefing is required');
  }

  // Calculate local origin from device positions
  const origin = calculateLocalOrigin(missionBriefing.drone_information);
  chatLogger.info(`Local origin calculated: lat=${origin.lat}, lng=${origin.lng}, alt=${origin.alt}`);

  // Deep clone to avoid mutating original
  const converted = JSON.parse(JSON.stringify(missionBriefing));

  // Add origin to the converted mission
  converted.origin_global = {
    lat: origin.lat,
    lng: origin.lng,
    alt: origin.alt,
  };

  // Convert device locations to XYZ (replace lat/lng with x/y/z)
  if (converted.drone_information) {
    for (const device of converted.drone_information) {
      if (device.location) {
        const enu = geodeticToENU(device.location.lat, device.location.lng, device.location.alt || 0, origin);
        device.location = {
          x: Number(enu.x.toFixed(3)),
          y: Number(enu.y.toFixed(3)),
          z: Number(enu.z.toFixed(3)),
        };
      }
    }
  }

  // Convert target elements positions to XYZ (replace lat/lng with x/y/z)
  if (converted.target_elements) {
    for (const element of converted.target_elements) {
      if (element.position) {
        const enu = geodeticToENU(element.position.lat, element.position.lng, element.position.alt || 0, origin);
        element.position = {
          x: Number(enu.x.toFixed(3)),
          y: Number(enu.y.toFixed(3)),
          z: Number(enu.z.toFixed(3)),
        };
      }
    }
  }

  // Convert points of interest positions to XYZ (replace lat/lng with x/y/z)
  if (converted.points_of_interest) {
    for (const poi of converted.points_of_interest) {
      if (poi.lat !== undefined && poi.lng !== undefined) {
        const enu = geodeticToENU(poi.lat, poi.lng, poi.alt || 0, origin);
        // Replace lat/lng/alt with x/y/z
        delete poi.lat;
        delete poi.lng;
        delete poi.alt;
        poi.position = {
          x: Number(enu.x.toFixed(3)),
          y: Number(enu.y.toFixed(3)),
          z: Number(enu.z.toFixed(3)),
        };
      }
    }
  }
  // convert obstacle elements positions to XYZ (replace lat/lng with x/y/z)
  if (converted.obstacle_elements) {
    for (const obstacle of converted.obstacle_elements) {
      if (obstacle.position) {
        const enu = geodeticToENU(obstacle.position.lat, obstacle.position.lng, obstacle.position.alt || 0, origin);
        obstacle.position = {
          x: Number(enu.x.toFixed(3)),
          y: Number(enu.y.toFixed(3)),
          z: Number(enu.z.toFixed(3)),
        };
      }
    }
  }

  chatLogger.info('Mission briefing converted to XYZ coordinates');
  return converted;
}

export { convertMissionBriefingToXYZ, convertMissionXYZToLatLong, geodeticToENU, ENUToGeodetic, calculateLocalOrigin };
