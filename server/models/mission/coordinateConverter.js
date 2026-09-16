import { chatLogger } from '../../common/logger.js';
import { devicesController } from '../../controllers/devices.js';
import { positionsController } from '../../controllers/positions.js';
import { resolveInspectionTargets, resolveObstaclesInBounds } from '../markers/inspectionTargets.js';

// Constants for geodetic calculations
const EARTH_RADIUS_M = 6378137.0; // WGS84 equatorial radius in meters
const DEG_TO_RAD = Math.PI / 180;
const ECCENTRICITY_SQ = 0.00669438; // WGS84 first eccentricity squared

// Targets/devices are anchored at their center point — without a margin, half of a
// physically-sized object (turbine, tower) could sit outside the mission boundary.
// Two margins, both in meters: TRAJECTORY is what's actually returned as the
// mission's flight boundary (room between an object's edge and free space).
// OBSTACLE_SEARCH is larger and internal-only — it widens the catalog lookup so an
// obstacle whose center falls just outside the trajectory boundary, but whose body
// would still reach into it, doesn't go unnoticed.
const TRAJECTORY_MARGIN_M = 150;
const OBSTACLE_SEARCH_MARGIN_M = 200;

// Above this separation between devices/targets, the mission area is
// considered unreasonable (likely a bad LLM-supplied coordinate) rather than
// a legitimately large inspection.
const MAX_MISSION_SPAN_M = 10000;

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

  return { x: Math.round(x * 100) / 100, y: Math.round(y * 100) / 100, z: Math.round(z * 100) / 100 };
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
 * Expand an ENU bounding box outward by a flat margin in meters, on the
 * horizontal plane only (z is left untouched).
 * @param {{min:{x:number,y:number,z:number}, max:{x:number,y:number,z:number}}} box
 * @param {number} marginM
 * @returns {{min:{x:number,y:number,z:number}, max:{x:number,y:number,z:number}}}
 */
function expandENUBox(box, marginM) {
  return {
    min: { x: box.min.x - marginM, y: box.min.y - marginM, z: box.min.z },
    max: { x: box.max.x + marginM, y: box.max.y + marginM, z: box.max.z },
  };
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

  if (!missionData.global_origin) {
    throw new Error('Mission data must contain global_origin for coordinate conversion');
  }

  const origin = missionData.global_origin;
  chatLogger.info(`Using origin for conversion: lat=${origin.lat}, lng=${origin.lng}, alt=${origin.alt}`);

  // Deep clone to avoid mutating original
  const converted = JSON.parse(JSON.stringify(missionData));

  // Remove global_origin from converted mission (not needed in lat/lng format)
  delete converted.global_origin;

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
              Number(geodetic.lat.toFixed(10)),
              Number(geodetic.lng.toFixed(10)),
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
 * @returns {Object} Mission briefing with XYZ coordinates and global_origin
 */
async function convertMissionBriefingToXYZ(missionBriefing) {
  if (!missionBriefing) {
    throw Object.assign(new Error('Mission briefing is required'), { status: 400 });
  }
  const { selected_devices, targets } = missionBriefing;

  if (!selected_devices || selected_devices.length === 0) {
    throw Object.assign(new Error('No devices available to calculate local origin'), { status: 400 });
  }

  // Resolve each LLM-supplied device against the DB by name (unique, and the
  // field an LLM is least likely to misremember vs. the numeric id). id and
  // category are cross-checked, not trusted blindly, so a hallucinated
  // id/category is rejected rather than silently used. The mismatch error
  // deliberately does NOT echo back the real device's id/category — the LLM
  // would otherwise pick up an unrelated device's data from the error and
  // treat it as required context. All mismatches are collected before
  // throwing so the LLM can fix every bad entry in one retry.
  const deviceErrors = [];
  const devices = await Promise.all(
    selected_devices.map(async (requested) => {
      const dbDevice = await devicesController.getByName(requested.name);
      if (!dbDevice) {
        deviceErrors.push(`Device "${requested.name}" not found.`);
        return null;
      }
      if (String(dbDevice.id) !== String(requested.id) || dbDevice.category !== requested.category) {
        deviceErrors.push(
          `Device "${requested.name}" is invalid: no device exists with that exact combination of name, id and category. Re-check the device list before retrying.`
        );
        return null;
      }

      const position = await positionsController.getByDeviceId(dbDevice.id);
      if (!position || position.latitude === undefined || position.longitude === undefined) {
        deviceErrors.push(`Device "${requested.name}" has no known position.`);
        return null;
      }

      return {
        id: dbDevice.id,
        name: dbDevice.name,
        category: dbDevice.category,
        batteryLevel: position.attributes?.batteryLevel,
        location: {
          lat: position.latitude,
          lng: position.longitude,
          alt: position.altitude || 0,
        },
      };
    })
  );

  if (deviceErrors.length > 0) {
    throw Object.assign(new Error(`Invalid selected_devices:\n${deviceErrors.join('\n')}`), { status: 400 });
  }

  // Local origin from the resolved (real, DB-backed) device positions — needed
  // before resolving targets, since target lat/lng gets converted into this
  // same ENU frame.
  const origin = calculateLocalOrigin(devices);
  chatLogger.info(`Local origin calculated: lat=${origin.lat}, lng=${origin.lng}, alt=${origin.alt}`);

  if (!targets || targets.length === 0) {
    throw Object.assign(new Error('No targets provided'), { status: 400 });
  }

  // Resolve each LLM-supplied target against the catalog by id (ElementItem.name
  // isn't unique, unlike devices, so id is the only safe lookup key here).
  // name/group/type are cross-checked against the catalog record for that id;
  // a hallucinated id or a name/group/type that doesn't match its own id is
  // reported back instead of silently used. No globalOrigin is passed, so
  // position comes back raw geodetic — converted to XYZ further below, same
  // as devices.
  const {
    targets: resolvedTargets,
    notFound: targetsNotFound,
    mismatched: targetsMismatched,
  } = await resolveInspectionTargets(targets);

  const targetErrors = [...targetsNotFound.map((id) => `Target id=${id} not found.`), ...targetsMismatched];
  if (targetErrors.length > 0) {
    throw Object.assign(new Error(`Invalid targets:\n${targetErrors.join('\n')}`), { status: 400 });
  }

  // Mission boundaries: bounding box (SW/NE corners) that contains every
  // resolved device and target center point. Devices are constrained to
  // operate within this box (once margined below).
  const boundaryPoints = [
    ...devices.map((device) => device.location),
    ...resolvedTargets.map((target) => target.position),
  ];
  const boundaries = {
    min: {
      lat: Math.min(...boundaryPoints.map((point) => point.lat)),
      lng: Math.min(...boundaryPoints.map((point) => point.lng)),
    },
    max: {
      lat: Math.max(...boundaryPoints.map((point) => point.lat)),
      lng: Math.max(...boundaryPoints.map((point) => point.lng)),
    },
  };

  // convert device positions to XYZ coordinates
  const devicesXYZ = devices.map((device) => ({
    ...device,
    location: geodeticToENU(device.location.lat, device.location.lng, device.location.alt, origin),
  }));

  // convert target positions to XYZ coordinates
  const targetsXYZ = resolvedTargets.map((target) => ({
    ...target,
    position: geodeticToENU(target.position.lat, target.position.lng, target.position.alt, origin),
  }));

  // convert boundary positions to XYZ coordinates, then apply both margins
  const roundENU = ({ x, y, z }) => ({ x: Math.round(x), y: Math.round(y), z: Math.round(z) });
  const rawBoundariesXYZ = {
    min: roundENU(geodeticToENU(boundaries.min.lat, boundaries.min.lng, 0, origin)),
    max: roundENU(geodeticToENU(boundaries.max.lat, boundaries.max.lng, 0, origin)),
  };

  // Per-device distance to its nearest target — a device far from every target
  // is the actual failure mode (LLM picked a device outside the inspection
  // area), which a single whole-mission bounding-box span can't identify.
  const deviceDistances = devicesXYZ.map((device) => ({
    device,
    nearestTargetDistanceM: Math.min(
      ...targetsXYZ.map((target) =>
        Math.hypot(target.position.x - device.location.x, target.position.y - device.location.y)
      )
    ),
  }));
  const devicesOutOfRange = deviceDistances.filter(
    ({ nearestTargetDistanceM }) => nearestTargetDistanceM > MAX_MISSION_SPAN_M
  );

  if (devicesOutOfRange.length > 0) {
    const missionSpanM = Math.max(...devicesOutOfRange.map((d) => d.nearestTargetDistanceM));
    throw Object.assign(
      new Error(
        `${devicesOutOfRange.length} of ${devicesXYZ.length} devices are too far from the targets to perform the inspection. The maximum separation found between a device and its nearest target is ${(missionSpanM / 1000).toFixed(2)} km, and the maximum allowed is ${MAX_MISSION_SPAN_M / 1000} km.`
      ),
      { status: 400 }
    );
  }

  const trajectoryBoundariesXYZ = expandENUBox(rawBoundariesXYZ, TRAJECTORY_MARGIN_M);
  const obstacleSearchBoundariesXYZ = expandENUBox(rawBoundariesXYZ, OBSTACLE_SEARCH_MARGIN_M);

  // Obstacles: catalog elements inside the (wider) obstacle-search box that
  // aren't already mission targets. There's no computed geometry for them
  // yet, so they're handed to the LLM the same way targets are — it reasons
  // over description/attributes itself. The search box is converted back to
  // geodetic because ElementItems are queried by lat/lng in the DB, not ENU.
  const obstacleSearchMin = ENUToGeodetic(
    obstacleSearchBoundariesXYZ.min.x,
    obstacleSearchBoundariesXYZ.min.y,
    0,
    origin
  );
  const obstacleSearchMax = ENUToGeodetic(
    obstacleSearchBoundariesXYZ.max.x,
    obstacleSearchBoundariesXYZ.max.y,
    0,
    origin
  );
  const obstacles = await resolveObstaclesInBounds(
    {
      minLat: Math.min(obstacleSearchMin.lat, obstacleSearchMax.lat),
      maxLat: Math.max(obstacleSearchMin.lat, obstacleSearchMax.lat),
      minLng: Math.min(obstacleSearchMin.lng, obstacleSearchMax.lng),
      maxLng: Math.max(obstacleSearchMin.lng, obstacleSearchMax.lng),
    },
    resolvedTargets.map((target) => target.id),
    origin
  );

  const converted = {
    global_origin: {
      lat: origin.lat,
      lng: origin.lng,
      alt: origin.alt,
    },
    devices: devicesXYZ,
    targets: targetsXYZ,
    boundaries: trajectoryBoundariesXYZ,
    obstacles,
  };

  chatLogger.info('Mission briefing converted to XYZ coordinates');
  return converted;
}

export { convertMissionBriefingToXYZ, convertMissionXYZToLatLong, geodeticToENU, ENUToGeodetic, calculateLocalOrigin };
