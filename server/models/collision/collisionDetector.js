/**
 * Collision Detection for UAV Mission Routes
 * Detects collisions between flight paths and obstacles
 */

import logger from '../../common/logger.js';
import {
  segmentIntersectsAABB,
  segmentIntersectsCylinder,
  cylinderFromObstacleZone,
  pointInAABB,
  pointInCylinder,
  distance3D,
  interpolateSegment,
} from './geometry.js';

/**
 * @typedef {import('./geometry.js').Point3D} Point3D
 * @typedef {import('./geometry.js').AABB} AABB
 * @typedef {import('./geometry.js').Segment} Segment
 */

/**
 * @typedef {Object} Obstacle
 * @property {string} name - Obstacle identifier
 * @property {string} type - Obstacle type (e.g., 'windTurbine')
 * @property {Point3D} position - Center position
 * @property {Object} zones - Zone definitions
 * @property {string} zones.exclusion_zone - Exclusion zone description
 * @property {string} zones.caution_zone - Caution zone description
 * @property {string} zones.safe_zone - Safe zone description
 * @property {string[]} safe_passages - Pre-approved safe passages
 * @property {AABB} aabb - Axis-aligned bounding box
 * @property {string} [metadata] - Additional info
 */

/**
 * @typedef {Object} Waypoint
 * @property {string} type - 'inspection' | 'transit' | 'takeoff' | 'landing'
 * @property {Point3D|number[]} pos - Position (object or [x, y, z] array)
 * @property {number} [yaw] - Heading in degrees
 * @property {number} [speed] - Speed in m/s
 * @property {string} [target_id] - Target being inspected
 * @property {string} [notes] - Additional notes
 */

/**
 * @typedef {Object} CollisionResult
 * @property {boolean} hasCollision - Whether collision was detected
 * @property {string} obstacleName - Name of obstacle
 * @property {string} obstacleType - Type of obstacle
 * @property {string} zoneType - 'exclusion' | 'caution'
 * @property {number} segmentIndex - Index of colliding segment (start waypoint index)
 * @property {Point3D} collisionPoint - Approximate collision point
 * @property {number} penetrationDepth - How far into the zone (meters)
 * @property {Obstacle} obstacle - Full obstacle data
 */

/**
 * @typedef {Object} RouteValidationResult
 * @property {boolean} valid - Whether route is collision-free
 * @property {CollisionResult[]} collisions - List of detected collisions
 * @property {CollisionResult[]} warnings - List of caution zone entries
 * @property {Object} summary - Summary statistics
 */

// Safety margins (meters)
const SAFETY_MARGINS = {
  EXCLUSION: 2.0, // Extra margin for exclusion zones
  CAUTION: 0, // No extra margin for caution (it's already a buffer)
  WAYPOINT: 1.0, // Margin for waypoint position checks
};

/**
 * Normalize waypoint position to Point3D object
 * @param {Point3D|number[]} pos
 * @returns {Point3D}
 */
function normalizePosition(pos) {
  if (Array.isArray(pos)) {
    return { x: pos[0], y: pos[1], z: pos[2] };
  }
  return pos;
}

/**
 * Check if a single waypoint collides with an obstacle
 * @param {Waypoint} waypoint
 * @param {Obstacle} obstacle
 * @returns {CollisionResult|null}
 */
function checkWaypointCollision(waypoint, obstacle) {
  const pos = normalizePosition(waypoint.pos);

  // First check AABB (fast rejection)
  if (!pointInAABB(pos, obstacle.aabb, SAFETY_MARGINS.WAYPOINT)) {
    return null;
  }

  // Check exclusion zone (cylinder)
  const exclusionCylinder = cylinderFromObstacleZone(obstacle, 'exclusion_zone');
  if (exclusionCylinder && pointInCylinder(pos, exclusionCylinder, SAFETY_MARGINS.EXCLUSION)) {
    return {
      hasCollision: true,
      obstacleName: obstacle.name,
      obstacleType: obstacle.type,
      zoneType: 'exclusion',
      segmentIndex: -1, // Single point, no segment
      collisionPoint: pos,
      penetrationDepth: exclusionCylinder.radius - distance3D(pos, exclusionCylinder.center),
      // obstacle,
    };
  }

  // Check caution zone
  const cautionCylinder = cylinderFromObstacleZone(obstacle, 'caution_zone');
  if (cautionCylinder && pointInCylinder(pos, cautionCylinder, SAFETY_MARGINS.CAUTION)) {
    return {
      hasCollision: false, // Caution is warning, not collision
      obstacleName: obstacle.name,
      obstacleType: obstacle.type,
      zoneType: 'caution',
      segmentIndex: -1,
      collisionPoint: pos,
      penetrationDepth: cautionCylinder.radius - distance3D(pos, cautionCylinder.center),
      // obstacle,
    };
  }

  return null;
}

/**
 * Check if a segment between two waypoints collides with an obstacle
 * @param {Waypoint} wp1 - Start waypoint
 * @param {Waypoint} wp2 - End waypoint
 * @param {number} segmentIndex - Index of segment in route
 * @param {Obstacle} obstacle
 * @returns {{collision: CollisionResult|null, warning: CollisionResult|null}}
 */
function checkSegmentCollision(wp1, wp2, segmentIndex, obstacle) {
  const start = normalizePosition(wp1.pos);
  const end = normalizePosition(wp2.pos);
  const segment = { start, end };

  let collision = null;
  let warning = null;

  // Quick AABB rejection test
  // Use a larger margin to account for caution zone which may extend beyond AABB
  // The AABB is typically sized for exclusion zone, but caution zone can be larger/taller
  const cautionCylinder = cylinderFromObstacleZone(obstacle, 'caution_zone');
  const exclusionCylinder = cylinderFromObstacleZone(obstacle, 'exclusion_zone');

  // Calculate expanded AABB margin: max of caution zone dimensions beyond AABB
  let aabbMargin = SAFETY_MARGINS.EXCLUSION;
  if (cautionCylinder && obstacle.aabb) {
    // Caution zone may extend higher and wider than AABB
    const heightDiff = cautionCylinder.center.z + cautionCylinder.height - obstacle.aabb.max_point.z;
    const radiusDiff =
      cautionCylinder.radius -
      Math.max(
        (obstacle.aabb.max_point.x - obstacle.aabb.min_point.x) / 2,
        (obstacle.aabb.max_point.y - obstacle.aabb.min_point.y) / 2
      );
    aabbMargin = Math.max(aabbMargin, heightDiff, radiusDiff);
  }

  const aabbResult = segmentIntersectsAABB(segment, obstacle.aabb, aabbMargin);
  if (!aabbResult.intersects) {
    return { collision: null, warning: null };
  }

  // Check exclusion zone (cylinder) - already parsed above for AABB margin calculation
  if (exclusionCylinder) {
    const exclusionResult = segmentIntersectsCylinder(segment, exclusionCylinder, SAFETY_MARGINS.EXCLUSION);

    if (exclusionResult.intersects) {
      collision = {
        hasCollision: true,
        obstacleName: obstacle.name,
        obstacleType: obstacle.type,
        zoneType: 'exclusion',
        segmentIndex,
        collisionPoint: exclusionResult.closestPoint,
        penetrationDepth: exclusionCylinder.radius - exclusionResult.distance,
        // obstacle,
      };
    }
  }

  // Check caution zone (even if exclusion collision found, for complete reporting)
  // cautionCylinder already parsed above for AABB margin calculation
  if (cautionCylinder && !collision) {
    const cautionResult = segmentIntersectsCylinder(segment, cautionCylinder, SAFETY_MARGINS.CAUTION);

    if (cautionResult.intersects) {
      warning = {
        hasCollision: false,
        obstacleName: obstacle.name,
        obstacleType: obstacle.type,
        zoneType: 'caution',
        segmentIndex,
        collisionPoint: cautionResult.closestPoint,
        penetrationDepth: cautionCylinder.radius - cautionResult.distance,
        // obstacle,
      };
    }
  }

  return { collision, warning };
}

/**
 * Validate a complete route against all obstacles
 * @param {Waypoint[]} waypoints - Array of waypoints
 * @param {Obstacle[]} obstacles - Array of obstacles
 * @returns {RouteValidationResult}
 */
export function validateRoute(waypoints, obstacles) {
  const collisions = [];
  const warnings = [];

  if (!waypoints || waypoints.length === 0) {
    return {
      valid: true,
      collisions: [],
      warnings: [],
      summary: { totalWaypoints: 0, totalSegments: 0, collisionCount: 0, warningCount: 0 },
    };
  }

  if (!obstacles || obstacles.length === 0) {
    return {
      valid: true,
      collisions: [],
      warnings: [],
      summary: {
        totalWaypoints: waypoints.length,
        totalSegments: waypoints.length - 1,
        collisionCount: 0,
        warningCount: 0,
      },
    };
  }

  // Check each waypoint
  for (let i = 0; i < waypoints.length; i++) {
    const wp = waypoints[i];

    for (const obstacle of obstacles) {
      const result = checkWaypointCollision(wp, obstacle);
      if (result) {
        result.segmentIndex = i;
        if (result.zoneType === 'exclusion') {
          collisions.push(result);
        } else {
          warnings.push(result);
        }
      }
    }
  }

  // Check each segment between consecutive waypoints
  for (let i = 0; i < waypoints.length - 1; i++) {
    const wp1 = waypoints[i];
    const wp2 = waypoints[i + 1];

    for (const obstacle of obstacles) {
      const { collision, warning } = checkSegmentCollision(wp1, wp2, i, obstacle);

      if (collision) {
        collisions.push(collision);
      }
      if (warning) {
        warnings.push(warning);
      }
    }
  }

  // Sort by segment index
  collisions.sort((a, b) => a.segmentIndex - b.segmentIndex);
  warnings.sort((a, b) => a.segmentIndex - b.segmentIndex);

  return {
    valid: collisions.length === 0,
    collisions,
    warnings,
    summary: {
      totalWaypoints: waypoints.length,
      totalSegments: waypoints.length - 1,
      collisionCount: collisions.length,
      warningCount: warnings.length,
    },
  };
}

/**
 * Validate a complete mission (multiple routes) against obstacles
 * @param {Object} mission - Mission object with routes
 * @param {Object[]} mission.route - Array of route objects
 * @param {Obstacle[]} obstacles - Array of obstacles
 * @returns {Object} Validation results per route
 */
export function validateMission(mission, obstacles) {
  const results = {
    valid: true,
    routes: [],
    totalCollisions: 0,
    totalWarnings: 0,
  };

  if (!mission?.route || !Array.isArray(mission.route)) {
    logger.warn('[CollisionDetector] Mission has no routes');
    return results;
  }

  for (const route of mission.route) {
    const routeResult = validateRoute(route.wp || [], obstacles);

    results.routes.push({
      routeId: route.id,
      routeName: route.name,
      uav: route.uav,
      ...routeResult,
    });

    if (!routeResult.valid) {
      results.valid = false;
    }

    results.totalCollisions += routeResult.collisions.length;
    results.totalWarnings += routeResult.warnings.length;
  }

  logger.info(
    `[CollisionDetector] Mission validation: valid=${results.valid}, ` +
      `collisions=${results.totalCollisions}, warnings=${results.totalWarnings}`
  );

  return results;
}

/**
 * Find all obstacles that a specific segment collides with
 * @param {Point3D} start - Segment start
 * @param {Point3D} end - Segment end
 * @param {Obstacle[]} obstacles
 * @returns {Obstacle[]} Colliding obstacles
 */
export function findCollidingObstacles(start, end, obstacles) {
  const segment = { start, end };
  const colliding = [];

  for (const obstacle of obstacles) {
    // Quick AABB check
    const aabbResult = segmentIntersectsAABB(segment, obstacle.aabb, SAFETY_MARGINS.EXCLUSION);
    if (!aabbResult.intersects) continue;

    // Cylinder check
    const cylinder = cylinderFromObstacleZone(obstacle, 'exclusion_zone');
    if (cylinder) {
      const result = segmentIntersectsCylinder(segment, cylinder, SAFETY_MARGINS.EXCLUSION);
      if (result.intersects) {
        colliding.push(obstacle);
      }
    }
  }

  return colliding;
}

/**
 * Get detailed collision report as formatted string
 * @param {RouteValidationResult} result
 * @returns {string}
 */
export function formatCollisionReport(result) {
  const lines = [];
  lines.push('=== COLLISION DETECTION REPORT ===');
  lines.push(`Status: ${result.valid ? 'VALID (No collisions)' : 'INVALID (Collisions detected)'}`);
  lines.push(`Total waypoints: ${result.summary.totalWaypoints}`);
  lines.push(`Total segments: ${result.summary.totalSegments}`);
  lines.push(`Collisions: ${result.summary.collisionCount}`);
  lines.push(`Warnings: ${result.summary.warningCount}`);
  lines.push('');

  if (result.collisions.length > 0) {
    lines.push('--- COLLISIONS (Exclusion Zone Violations) ---');
    for (const c of result.collisions) {
      lines.push(
        `  [Segment ${c.segmentIndex}] ${c.obstacleName} (${c.obstacleType}): ` +
          `point=(${c.collisionPoint.x.toFixed(1)}, ${c.collisionPoint.y.toFixed(1)}, ${c.collisionPoint.z.toFixed(1)}), ` +
          `penetration=${c.penetrationDepth.toFixed(1)}m`
      );
    }
    lines.push('');
  }

  if (result.warnings.length > 0) {
    lines.push('--- WARNINGS (Caution Zone Entries) ---');
    for (const w of result.warnings) {
      lines.push(`  [Segment ${w.segmentIndex}] ${w.obstacleName}: entering caution zone`);
    }
  }

  return lines.join('\n');
}
