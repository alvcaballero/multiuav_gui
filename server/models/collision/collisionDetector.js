/**
 * Collision Detection for UAV Mission Routes
 * Detects collisions between flight paths and obstacles
 */

import { logger } from '../../common/logger.js';
import {
  segmentIntersectsAABB,
  pointInAABB,
  pointInCylinder,
  pointInOBB,
  segmentIntersectsCylinder,
  segmentIntersectsOBB,
  obstacleAABB,
  obstacleCylinder,
  obstacleOBB,
  obstacleCenter,
  distance3D,
  closestPointsBetweenSegments,
} from './geometry.js';

/**
 * @typedef {import('./geometry.js').Point3D} Point3D
 * @typedef {import('./geometry.js').AABB} AABB
 * @typedef {import('./geometry.js').Segment} Segment
 * @typedef {import('./geometry.js').Obstacle} Obstacle
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

/**
 * @typedef {Object} InterRouteCollision
 * @property {{id: number, name: string, uav: string, segmentIndex: number}} routeA
 * @property {{id: number, name: string, uav: string, segmentIndex: number}} routeB
 * @property {Point3D} point - Point of closest approach between the two segments
 * @property {number} distance - Spatial distance at closest approach (meters)
 * @property {number} timeA - Estimated time routeA's UAV reaches the point (seconds from mission start)
 * @property {number} timeB - Estimated time routeB's UAV reaches the point (seconds from mission start)
 * @property {number} timeDiff - |timeA - timeB| (seconds)
 */

// Safety margins (meters), applied on top of each obstacle's own safety_margin
const SAFETY_MARGINS = {
  EXCLUSION: 0, // safety_margin already includes the required clearance
  CAUTION_EXTRA: 5.0, // Extra margin beyond safety_margin that triggers a warning instead of a hard collision
  WAYPOINT: 1.0, // Margin for waypoint position checks
};

// Defaults for UAV-to-UAV route crossing checks (independent from obstacle margins above)
const INTER_ROUTE_DEFAULTS = {
  SPATIAL_THRESHOLD: 10, // meters - routes closer than this at closest approach are "the same point"
  TIME_WINDOW: 10, // seconds - both UAVs reaching that point within this window is a collision risk
  FALLBACK_SPEED: 5, // m/s - used when a route has neither idle_vel nor max_vel set
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

  // First check AABB (fast rejection), expanded to the caution margin so we
  // don't reject points that would only trigger a caution warning.
  if (!pointInAABB(pos, obstacleAABB(obstacle, SAFETY_MARGINS.WAYPOINT), SAFETY_MARGINS.CAUTION_EXTRA)) {
    return null;
  }

  if (obstacle.geometry_type === 'rectangle') {
    const obb = obstacleOBB(obstacle, SAFETY_MARGINS.WAYPOINT);
    if (pointInOBB(pos, obb)) {
      const center = obstacleCenter(obstacle);
      return {
        hasCollision: true,
        obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
        obstacleType: obstacle.geometry_type,
        zoneType: 'exclusion',
        segmentIndex: -1, // Single point, no segment
        collisionPoint: pos,
        penetrationDepth: distance3D(pos, center), // approximate: distance to obstacle center
        obstacle,
      };
    }
    const cautionObb = obstacleOBB(obstacle, SAFETY_MARGINS.WAYPOINT + SAFETY_MARGINS.CAUTION_EXTRA);
    if (pointInOBB(pos, cautionObb)) {
      const center = obstacleCenter(obstacle);
      return {
        hasCollision: false, // Caution is warning, not collision
        obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
        obstacleType: obstacle.geometry_type,
        zoneType: 'caution',
        segmentIndex: -1,
        collisionPoint: pos,
        penetrationDepth: distance3D(pos, center),
        obstacle,
      };
    }
    return null;
  }

  const exclusionCylinder = obstacleCylinder(obstacle, SAFETY_MARGINS.WAYPOINT);
  if (pointInCylinder(pos, exclusionCylinder, SAFETY_MARGINS.EXCLUSION)) {
    return {
      hasCollision: true,
      obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
      obstacleType: obstacle.geometry_type,
      zoneType: 'exclusion',
      segmentIndex: -1, // Single point, no segment
      collisionPoint: pos,
      penetrationDepth: exclusionCylinder.radius - distance3D(pos, exclusionCylinder.center),
      obstacle,
    };
  }

  const cautionCylinder = obstacleCylinder(obstacle, SAFETY_MARGINS.WAYPOINT + SAFETY_MARGINS.CAUTION_EXTRA);
  if (pointInCylinder(pos, cautionCylinder, SAFETY_MARGINS.EXCLUSION)) {
    return {
      hasCollision: false, // Caution is warning, not collision
      obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
      obstacleType: obstacle.geometry_type,
      zoneType: 'caution',
      segmentIndex: -1,
      collisionPoint: pos,
      penetrationDepth: cautionCylinder.radius - distance3D(pos, cautionCylinder.center),
      obstacle,
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

  // Quick AABB rejection test, expanded to the caution margin so we don't
  // reject segments that would only trigger a caution warning.
  const aabbResult = segmentIntersectsAABB(segment, obstacleAABB(obstacle, SAFETY_MARGINS.CAUTION_EXTRA));
  if (!aabbResult.intersects) {
    return { collision: null, warning: null };
  }

  if (obstacle.geometry_type === 'rectangle') {
    const obb = obstacleOBB(obstacle, SAFETY_MARGINS.EXCLUSION);
    const exclusionResult = segmentIntersectsOBB(segment, obb);

    if (exclusionResult.intersects) {
      collision = {
        hasCollision: true,
        obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
        obstacleType: obstacle.geometry_type,
        zoneType: 'exclusion',
        segmentIndex,
        collisionPoint: start,
        penetrationDepth: obstacle.safety_margin,
        obstacle,
      };
    } else {
      const cautionObb = obstacleOBB(obstacle, SAFETY_MARGINS.EXCLUSION + SAFETY_MARGINS.CAUTION_EXTRA);
      const cautionResult = segmentIntersectsOBB(segment, cautionObb);
      if (cautionResult.intersects) {
        warning = {
          hasCollision: false,
          obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
          obstacleType: obstacle.geometry_type,
          zoneType: 'caution',
          segmentIndex,
          collisionPoint: start,
          penetrationDepth: obstacle.safety_margin,
          obstacle,
        };
      }
    }

    return { collision, warning };
  }

  const exclusionCylinder = obstacleCylinder(obstacle, SAFETY_MARGINS.EXCLUSION);
  const exclusionResult = segmentIntersectsCylinder(segment, exclusionCylinder);

  if (exclusionResult.intersects) {
    collision = {
      hasCollision: true,
      obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
      obstacleType: obstacle.geometry_type,
      zoneType: 'exclusion',
      segmentIndex,
      collisionPoint: exclusionResult.closestPoint,
      penetrationDepth: exclusionCylinder.radius - exclusionResult.distance,
      obstacle,
    };
  } else {
    const cautionCylinder = obstacleCylinder(obstacle, SAFETY_MARGINS.EXCLUSION + SAFETY_MARGINS.CAUTION_EXTRA);
    const cautionResult = segmentIntersectsCylinder(segment, cautionCylinder);

    if (cautionResult.intersects) {
      warning = {
        hasCollision: false,
        obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
        obstacleType: obstacle.geometry_type,
        zoneType: 'caution',
        segmentIndex,
        collisionPoint: cautionResult.closestPoint,
        penetrationDepth: cautionCylinder.radius - cautionResult.distance,
        obstacle,
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
      summary: { totalWaypoints: 0, totalSegments: 0, collisionCount: 0, warningCount: 0, totalDistance: 0 },
    };
  }

  // Total route distance (computed once, reused by every return path below)
  let totalDistance = 0;
  for (let i = 0; i < waypoints.length - 1; i++) {
    const p1 = normalizePosition(waypoints[i].pos);
    const p2 = normalizePosition(waypoints[i + 1].pos);
    totalDistance += distance3D(p1, p2);
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
        totalDistance,
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
      totalDistance, // meters
    },
  };
}

/**
 * Estimate cumulative arrival time (seconds from route start) at each waypoint.
 * Speed is taken from the route's idle_vel (falls back to max_vel, then a fixed
 * default) rather than per-waypoint speed, since idle_vel is the conservative
 * inspection-pass speed shared across the whole route.
 * @param {Object} route - Route object with wp[] and attributes
 * @returns {number[]} times[i] = seconds to reach waypoint i (times[0] = 0)
 */
function computeRouteTimings(route) {
  const wp = route.wp || [];
  const speed = route.attributes?.idle_vel ?? route.attributes?.max_vel ?? INTER_ROUTE_DEFAULTS.FALLBACK_SPEED;

  const times = [0];
  for (let i = 0; i < wp.length - 1; i++) {
    const a = normalizePosition(wp[i].pos);
    const b = normalizePosition(wp[i + 1].pos);
    const segDuration = speed > 0 ? distance3D(a, b) / speed : 0;
    times.push(times[i] + segDuration);
  }
  return times;
}

/**
 * Find UAV-to-UAV collision risks between routes: pairs of segments (from
 * different UAVs) that pass close to each other in space AND whose estimated
 * arrival times at that point are close together.
 * @param {Object[]} routes - Array of route objects (mission.route)
 * @param {Object} [options]
 * @param {number} [options.spatialThreshold] - Meters; default INTER_ROUTE_DEFAULTS.SPATIAL_THRESHOLD
 * @param {number} [options.timeWindow] - Seconds; default INTER_ROUTE_DEFAULTS.TIME_WINDOW
 * @returns {InterRouteCollision[]} Sorted by timeDiff ascending (most dangerous first)
 */
export function findInterRouteCollisions(routes, options = {}) {
  const spatialThreshold = options.spatialThreshold ?? INTER_ROUTE_DEFAULTS.SPATIAL_THRESHOLD;
  const timeWindow = options.timeWindow ?? INTER_ROUTE_DEFAULTS.TIME_WINDOW;
  const conflicts = [];

  if (!Array.isArray(routes) || routes.length < 2) {
    return conflicts;
  }

  const timings = routes.map(computeRouteTimings);

  for (let i = 0; i < routes.length; i++) {
    const routeA = routes[i];
    const wpA = routeA.wp || [];
    if (wpA.length < 2) continue;

    for (let j = i + 1; j < routes.length; j++) {
      const routeB = routes[j];
      const wpB = routeB.wp || [];
      if (wpB.length < 2) continue;
      if (routeA.uav && routeB.uav && routeA.uav === routeB.uav) continue; // same UAV can't collide with itself

      for (let a = 0; a < wpA.length - 1; a++) {
        const segA = { start: normalizePosition(wpA[a].pos), end: normalizePosition(wpA[a + 1].pos) };

        for (let b = 0; b < wpB.length - 1; b++) {
          const segB = { start: normalizePosition(wpB[b].pos), end: normalizePosition(wpB[b + 1].pos) };

          const closest = closestPointsBetweenSegments(segA, segB);
          if (closest.distance > spatialThreshold) continue;

          const timeA = timings[i][a] + closest.tA * (timings[i][a + 1] - timings[i][a]);
          const timeB = timings[j][b] + closest.tB * (timings[j][b + 1] - timings[j][b]);
          const timeDiff = Math.abs(timeA - timeB);

          if (timeDiff <= timeWindow) {
            conflicts.push({
              routeA: { id: routeA.id, name: routeA.name, uav: routeA.uav, segmentIndex: a },
              routeB: { id: routeB.id, name: routeB.name, uav: routeB.uav, segmentIndex: b },
              point: closest.pointA,
              distance: closest.distance,
              timeA,
              timeB,
              timeDiff,
            });
          }
        }
      }
    }
  }

  return conflicts.sort((x, y) => x.timeDiff - y.timeDiff);
}

/**
 * Validate a complete mission (multiple routes) against obstacles, and check
 * routes against each other for UAV-to-UAV collision risk (see
 * findInterRouteCollisions).
 * @param {Object} mission - Mission object with routes
 * @param {Object[]} mission.route - Array of route objects
 * @param {Obstacle[]} obstacles - Array of obstacles
 * @param {Object} [options]
 * @param {Object} [options.interRoute] - Forwarded to findInterRouteCollisions
 * @returns {Object} Validation results per route, plus interRouteCollisions
 */
export function validateMissionCollission(mission, obstacles, options = {}) {
  const results = {
    valid: true,
    routes: [],
    totalCollisions: 0,
    totalWarnings: 0,
    totalDistance: 0,
    interRouteCollisions: [],
  };

  if (!mission?.route || !Array.isArray(mission.route)) {
    logger.warn('[CollisionDetector] Mission has no routes');
    return results;
  }

  for (const route of mission.route) {
    const routeResult = validateRoute(route.wp || [], obstacles || []);

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
    results.totalDistance += routeResult.summary.totalDistance;
  }

  results.interRouteCollisions = findInterRouteCollisions(mission.route, options.interRoute);
  if (results.interRouteCollisions.length > 0) {
    results.valid = false;
  }

  logger.info(
    `[CollisionDetector] Mission validation: valid=${results.valid}, ` +
      `collisions=${results.totalCollisions}, warnings=${results.totalWarnings}, ` +
      `interRouteCollisions=${results.interRouteCollisions.length}, ` +
      `totalDistance=${results.totalDistance.toFixed(1)}m`
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
    const aabbResult = segmentIntersectsAABB(segment, obstacleAABB(obstacle, SAFETY_MARGINS.EXCLUSION));
    if (!aabbResult.intersects) continue;

    const intersects =
      obstacle.geometry_type === 'rectangle'
        ? segmentIntersectsOBB(segment, obstacleOBB(obstacle, SAFETY_MARGINS.EXCLUSION)).intersects
        : segmentIntersectsCylinder(segment, obstacleCylinder(obstacle, SAFETY_MARGINS.EXCLUSION)).intersects;

    if (intersects) {
      colliding.push(obstacle);
    }
  }

  return colliding;
}

/**
 * Format a single collision/warning entry
 * @param {CollisionResult} c
 * @returns {string}
 */
function formatCollisionEntry(c) {
  const point = `point=(${c.collisionPoint.x.toFixed(1)}, ${c.collisionPoint.y.toFixed(1)}, ${c.collisionPoint.z.toFixed(1)})`;
  if (c.zoneType === 'exclusion') {
    return `  * Segment [${c.segmentIndex}]: Collision with ${c.obstacleName} at ${point} - Penetration: ${c.penetrationDepth.toFixed(1)}m`;
  }
  return `  * Segment [${c.segmentIndex}]: Warning near ${c.obstacleName} at ${point} - Proximity: ${c.penetrationDepth.toFixed(1)}m`;
}

/**
 * Format a per-route collision report section
 * @param {Object} routeResult - Route validation result with routeName, valid, collisions, warnings, summary
 * @returns {string}
 */
export function formatRouteReport(routeResult) {
  const lines = [];
  const name = routeResult.routeName || routeResult.uav || `Route ${routeResult.routeId}`;

  lines.push(`#### [ROUTE: ${name}]`);
  lines.push(`- **Status:** ${routeResult.valid ? 'VALID' : 'INVALID'}`);
  lines.push(`- Distance: ${routeResult.summary.totalDistance.toFixed(1)} (m)`);

  if (routeResult.collisions.length > 0) {
    lines.push('');
    lines.push('- **Critical Segments to Fix:**');
    for (const c of routeResult.collisions) {
      lines.push(formatCollisionEntry(c));
    }
  }

  // if (routeResult.warnings.length > 0) {
  //   lines.push('');
  //   lines.push('- **Warnings:**');
  //   for (const w of routeResult.warnings) {
  //     lines.push(formatCollisionEntry(w));
  //   }
  // }

  return lines.join('\n');
}

/**
 * Format a single inter-route (UAV-to-UAV) collision alert
 * @param {InterRouteCollision} c
 * @returns {string}
 */
function formatInterRouteEntry(c) {
  const point = `point=(${c.point.x.toFixed(1)}, ${c.point.y.toFixed(1)}, ${c.point.z.toFixed(1)})`;
  const uavA = c.routeA.uav || c.routeA.name || `route ${c.routeA.id}`;
  const uavB = c.routeB.uav || c.routeB.name || `route ${c.routeB.id}`;
  return (
    `  * ALERT: ${uavA} [seg ${c.routeA.segmentIndex}] and ${uavB} [seg ${c.routeB.segmentIndex}] ` +
    `cross paths at ${point} (distance=${c.distance.toFixed(1)}m) - ` +
    `${uavA}@${c.timeA.toFixed(1)}s vs ${uavB}@${c.timeB.toFixed(1)}s (Δt=${c.timeDiff.toFixed(1)}s) - ` +
    `possible collision, generate a detour`
  );
}

/**
 * Format full mission validation report
 * @param {Object} missionResult - Result from validateMissionCollission()
 * @returns {string}
 */
export function formatMissionReport(missionResult) {
  const lines = [];
  const status = missionResult.valid ? 'VALID (No collisions)' : 'INVALID (Collisions detected)';

  lines.push(`Status: ${status}`);
  lines.push(
    `**Total Collisions:** ${missionResult.totalCollisions} | **Total Warnings:** ${missionResult.totalWarnings}` +
      ` | **Inter-UAV Conflicts:** ${missionResult.interRouteCollisions?.length ?? 0}`
  );
  lines.push(`- totalDistance: ${missionResult.totalDistance.toFixed(1)} (m)`);

  if (missionResult.interRouteCollisions?.length > 0) {
    lines.push('');
    lines.push('#### [INTER-UAV ROUTE CONFLICTS]');
    for (const c of missionResult.interRouteCollisions) {
      lines.push(formatInterRouteEntry(c));
    }
  }

  for (const route of missionResult.routes) {
    lines.push('');
    lines.push(formatRouteReport(route));
  }

  return lines.join('\n');
}

/**
 * Get detailed collision report as formatted string (legacy per-route format)
 * @param {RouteValidationResult} result
 * @returns {string}
 */
export function formatCollisionReport(result) {
  return formatRouteReport(result);
}
