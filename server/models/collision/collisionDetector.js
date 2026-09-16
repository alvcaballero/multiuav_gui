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
  obstacleCenter,
  obstacleCylinder,
  obstacleOBB,
  distance3D,
  closestPointsBetweenSegments,
  penetrationDepthOBB,
  penetrationDepthCylinder,
  interpolateSegment,
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
 * @property {string} [target_id] - Catalog `obstacle_id` of the target being inspected - a
 *           numeric-looking ID, not a display name; resolve it against `collision_objects`
 *           (see `obstacleNameById` in {@link validateRoute}) before showing it to a human/LLM
 * @property {string} [notes] - Additional notes
 */

/**
 * @typedef {Object} CollisionResult
 * @property {boolean} hasCollision - Whether collision was detected
 * @property {string} obstacleName - Name of obstacle
 * @property {string} obstacleType - Type of obstacle
 * @property {string} zoneType - 'exclusion' | 'caution'
 * @property {number} segmentIndex - Index of colliding segment (start waypoint index)
 * @property {string} label - Human-readable identity of the colliding waypoint(s): the
 *           `target_id`/`type` of the single waypoint, or `"A -> B"` for a segment between
 *           different groups, or `"intra-group A"` when both endpoints belong to the same
 *           group - lets the caller tell a transit-leg collision from a chord crossing the
 *           group's own object without re-deriving it from segmentIndex
 * @property {Point3D} collisionPoint - Approximate collision point
 * @property {{xy: number, z: number}} penetrationDepth - How far into the zone, split into
 *           horizontal (xy, distance to the nearest side wall) and vertical (z, distance to
 *           the nearest floor/ceiling) so the shortest way out is clear (meters)
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
  EXCLUSION: 2.0, // safety_margin already includes the required clearance
  CAUTION_EXTRA: 5.0, // Extra margin beyond safety_margin that triggers a warning instead of a hard collision
  WAYPOINT: 2.0, // Margin for waypoint position checks
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
 * Margin to expand an obstacle's real geometry by for the CAUTION zone: its
 * own safety_margin plus an extra buffer. The exclusion (hard collision) zone
 * uses the real geometry alone - safety_margin is purely a caution/warning
 * buffer, not part of what counts as a collision.
 * @param {Obstacle} obstacle
 * @param {number} extra - Additional margin on top of safety_margin (meters)
 * @returns {number}
 */
function cautionMargin(obstacle, extra) {
  return Math.max(obstacle.safety_margin, extra);
}

/**
 * Identity label for a single waypoint: the display name of the inspection
 * target it belongs to (resolved from `target_id`, a catalog ID, via
 * `obstacleNameById`), or its type (transit/takeoff/landing) when it has no
 * target. Falls back to the raw `target_id` if it has no matching obstacle -
 * that itself signals a Step 1 gap (every target must get a collision_object).
 * @param {Waypoint} waypoint
 * @param {Map<string,string>} obstacleNameById - `obstacle_id -> obstacle_name`, from {@link validateRoute}
 * @returns {string}
 */
function waypointLabel(waypoint, obstacleNameById) {
  if (waypoint.target_id != null) {
    return obstacleNameById.get(String(waypoint.target_id)) ?? waypoint.target_id;
  }
  return waypoint.type ?? 'unknown';
}

/**
 * Identity label for a segment between two waypoints - see the `label` field
 * of {@link CollisionResult} for what each shape means.
 * @param {Waypoint} wp1
 * @param {Waypoint} wp2
 * @param {Map<string,string>} obstacleNameById
 * @returns {string}
 */
function segmentLabel(wp1, wp2, obstacleNameById) {
  const from = waypointLabel(wp1, obstacleNameById);
  const to = waypointLabel(wp2, obstacleNameById);
  return from === to ? `intra-group ${from}` : `${from} -> ${to}`;
}

/**
 * Check if a single waypoint collides with an obstacle
 * @param {Waypoint} waypoint
 * @param {Obstacle} obstacle
 * @param {Map<string,string>} obstacleNameById
 * @returns {CollisionResult|null}
 */
function checkWaypointCollision(waypoint, obstacle, obstacleNameById) {
  const pos = normalizePosition(waypoint.pos);

  // First check AABB (fast rejection), expanded to the full caution margin so
  // we don't reject points that would only trigger a caution warning.
  const maxCautionMargin = cautionMargin(obstacle, SAFETY_MARGINS.WAYPOINT + SAFETY_MARGINS.CAUTION_EXTRA);
  if (!pointInAABB(pos, obstacleAABB(obstacle, maxCautionMargin))) {
    return null;
  }

  if (obstacle.geometry_type === 'rectangle') {
    const obb = obstacleOBB(obstacle, SAFETY_MARGINS.WAYPOINT);
    if (pointInOBB(pos, obb)) {
      return {
        hasCollision: true,
        obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
        obstacleType: obstacle.geometry_type,
        zoneType: 'exclusion',
        segmentIndex: -1, // Single point, no segment
        label: waypointLabel(waypoint, obstacleNameById),
        collisionPoint: pos,
        penetrationDepth: penetrationDepthOBB(pos, obb),
        obstacle,
      };
    }
    const cautionObb = obstacleOBB(
      obstacle,
      cautionMargin(obstacle, SAFETY_MARGINS.WAYPOINT + SAFETY_MARGINS.CAUTION_EXTRA)
    );
    if (pointInOBB(pos, cautionObb)) {
      return {
        hasCollision: false, // Caution is warning, not collision
        obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
        obstacleType: obstacle.geometry_type,
        zoneType: 'caution',
        segmentIndex: -1,
        label: waypointLabel(waypoint, obstacleNameById),
        collisionPoint: pos,
        penetrationDepth: penetrationDepthOBB(pos, cautionObb),
        obstacle,
      };
    }
    return null;
  }

  const exclusionCylinder = obstacleCylinder(obstacle, SAFETY_MARGINS.WAYPOINT);
  if (pointInCylinder(pos, exclusionCylinder)) {
    return {
      hasCollision: true,
      obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
      obstacleType: obstacle.geometry_type,
      zoneType: 'exclusion',
      segmentIndex: -1, // Single point, no segment
      label: waypointLabel(waypoint, obstacleNameById),
      collisionPoint: pos,
      penetrationDepth: penetrationDepthCylinder(pos, exclusionCylinder),
      obstacle,
    };
  }

  const cautionCylinder = obstacleCylinder(
    obstacle,
    cautionMargin(obstacle, SAFETY_MARGINS.WAYPOINT + SAFETY_MARGINS.CAUTION_EXTRA)
  );
  if (pointInCylinder(pos, cautionCylinder)) {
    return {
      hasCollision: false, // Caution is warning, not collision
      obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
      obstacleType: obstacle.geometry_type,
      zoneType: 'caution',
      segmentIndex: -1,
      label: waypointLabel(waypoint, obstacleNameById),
      collisionPoint: pos,
      penetrationDepth: penetrationDepthCylinder(pos, cautionCylinder),
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
 * @param {Map<string,string>} obstacleNameById
 * @returns {{collision: CollisionResult|null, warning: CollisionResult|null}}
 */
function checkSegmentCollision(wp1, wp2, segmentIndex, obstacle, obstacleNameById) {
  const start = normalizePosition(wp1.pos);
  const end = normalizePosition(wp2.pos);
  const segment = { start, end };

  let collision = null;
  let warning = null;

  // Quick AABB rejection test, expanded to the full caution margin so we
  // don't reject segments that would only trigger a caution warning.
  const aabbResult = segmentIntersectsAABB(
    segment,
    obstacleAABB(obstacle, cautionMargin(obstacle, SAFETY_MARGINS.CAUTION_EXTRA))
  );
  if (!aabbResult.intersects) {
    return { collision: null, warning: null };
  }

  if (obstacle.geometry_type === 'rectangle') {
    const obb = obstacleOBB(obstacle, SAFETY_MARGINS.EXCLUSION);
    const exclusionResult = segmentIntersectsOBB(segment, obb);

    if (exclusionResult.intersects) {
      // Deepest point along the segment's in-box span, used to measure how far
      // the route actually cuts into the box rather than reporting a constant.
      const deepestT = (exclusionResult.tMin + exclusionResult.tMax) / 2;
      const deepestPoint = interpolateSegment(segment, deepestT);
      collision = {
        hasCollision: true,
        obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
        obstacleType: obstacle.geometry_type,
        zoneType: 'exclusion',
        segmentIndex,
        label: segmentLabel(wp1, wp2, obstacleNameById),
        collisionPoint: deepestPoint,
        penetrationDepth: penetrationDepthOBB(deepestPoint, obb),
        segmentStart: start,
        segmentEnd: end,
        obstacle,
      };
    } else {
      const cautionObb = obstacleOBB(
        obstacle,
        cautionMargin(obstacle, SAFETY_MARGINS.EXCLUSION + SAFETY_MARGINS.CAUTION_EXTRA)
      );
      const cautionResult = segmentIntersectsOBB(segment, cautionObb);
      if (cautionResult.intersects) {
        const deepestT = (cautionResult.tMin + cautionResult.tMax) / 2;
        const deepestPoint = interpolateSegment(segment, deepestT);
        warning = {
          hasCollision: false,
          obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
          obstacleType: obstacle.geometry_type,
          zoneType: 'caution',
          segmentIndex,
          label: segmentLabel(wp1, wp2, obstacleNameById),
          collisionPoint: deepestPoint,
          penetrationDepth: penetrationDepthOBB(deepestPoint, cautionObb),
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
      label: segmentLabel(wp1, wp2, obstacleNameById),
      collisionPoint: exclusionResult.closestPoint,
      penetrationDepth: penetrationDepthCylinder(exclusionResult.closestPoint, exclusionCylinder),
      segmentStart: start,
      segmentEnd: end,
      obstacle,
    };
  } else {
    const cautionCylinder = obstacleCylinder(
      obstacle,
      cautionMargin(obstacle, SAFETY_MARGINS.EXCLUSION + SAFETY_MARGINS.CAUTION_EXTRA)
    );
    const cautionResult = segmentIntersectsCylinder(segment, cautionCylinder);

    if (cautionResult.intersects) {
      warning = {
        hasCollision: false,
        obstacleName: obstacle.obstacle_name ?? obstacle.obstacle_id,
        obstacleType: obstacle.geometry_type,
        zoneType: 'caution',
        segmentIndex,
        label: segmentLabel(wp1, wp2, obstacleNameById),
        collisionPoint: cautionResult.closestPoint,
        penetrationDepth: penetrationDepthCylinder(cautionResult.closestPoint, cautionCylinder),
        obstacle,
      };
    }
  }

  return { collision, warning };
}

/**
 * Compute the total 3D distance along a route's waypoints.
 * Pure — independent of obstacles, does not participate in collision detection.
 * @param {Waypoint[]} waypoints - Array of waypoints
 * @returns {number} Total distance in meters
 */
export function computeRouteDistance(waypoints) {
  if (!waypoints || waypoints.length === 0) return 0;

  let totalDistance = 0;
  for (let i = 0; i < waypoints.length - 1; i++) {
    const p1 = normalizePosition(waypoints[i].pos);
    const p2 = normalizePosition(waypoints[i + 1].pos);
    totalDistance += distance3D(p1, p2);
  }
  return totalDistance;
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

  // target_id on a waypoint is a catalog obstacle_id (e.g. "23"), not a display
  // name - resolve it once here against the obstacle list (which includes an
  // entry per inspection target, per Step 1) instead of showing the raw ID.
  const obstacleNameById = new Map(obstacles.map((o) => [String(o.obstacle_id), o.obstacle_name ?? o.obstacle_id]));

  // Check each waypoint
  for (let i = 0; i < waypoints.length; i++) {
    const wp = waypoints[i];

    for (const obstacle of obstacles) {
      const result = checkWaypointCollision(wp, obstacle, obstacleNameById);
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
      const { collision, warning } = checkSegmentCollision(wp1, wp2, i, obstacle, obstacleNameById);

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
    const distance = computeRouteDistance(route.wp || []);

    results.routes.push({
      routeId: route.id,
      routeName: route.name,
      uav: route.uav,
      distance,
      ...routeResult,
    });

    if (!routeResult.valid) {
      results.valid = false;
    }

    results.totalCollisions += routeResult.collisions.length;
    results.totalWarnings += routeResult.warnings.length;
    results.totalDistance += distance;
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
 * Horizontal distance from a segment's start to an obstacle's center - the
 * rank key for the obstacles blocking one segment. The planner bypasses the
 * closest one and defers the rest, so the ORDER is what this drives; printing
 * it as `d=` alongside `from=` and `center=` also lets the reader check the
 * ranking against the numbers on the same rows.
 *
 * Measured in XY, not 3D: obstacles rise from the ground, so folding in the
 * altitude gap would rank a nearby obstacle behind a distant one purely
 * because the route happens to fly high over it.
 * @param {Point3D} start - Segment start
 * @param {Obstacle} obstacle
 * @returns {number} Distance in meters
 */
function distanceFromStart(start, obstacle) {
  const c = obstacleCenter(obstacle);
  return Math.hypot(c.x - start.x, c.y - start.y);
}

/**
 * `1` -> `1st`, `2` -> `2nd`, ... - the rank prefix that tells the planner
 * which obstacle on a blocked segment it must bypass first.
 * @param {number} n - 1-based rank
 * @returns {string}
 */
function ordinal(n) {
  if (n === 1) return '1st';
  if (n === 2) return '2nd';
  if (n === 3) return '3rd';
  return `${n}th`;
}

/**
 * Format one blocking obstacle as a line under its segment.
 *
 * Reports the obstacle's own geometry (center, real size, height) rather than
 * only the collision point: the planner needs the CENTER to generate bypass
 * candidates at a radius around it, and without it here it has to look the
 * obstacle back up in its own input - a step where it has been observed to
 * invent coordinates. `r`/`w x l` is the real footprint the exclusion test
 * actually uses (safety_margin is caution-only, see {@link cautionMargin}),
 * so `depth_xy` is measured against it and the two numbers stay comparable.
 * `h` is the raw obstacle height, NOT the cylinder's (which folds in the
 * margin) - the planner adds its own clearance on top for a vertical hop.
 * @param {CollisionResult} c
 * @param {number} rank - 1-based position along the segment
 * @returns {string}
 */
function formatObstacleEntry(c, rank) {
  const o = c.obstacle ?? {};
  const center = obstacleCenter(o);
  const size =
    o.geometry_type === 'rectangle'
      ? `${o.dimensions?.width ?? 0}x${o.dimensions?.length ?? 0} yaw=${o.yaw ?? 0}`
      : `r=${o.dimensions?.radius ?? 0}`;
  const d = c.segmentStart ? `d=${distanceFromStart(c.segmentStart, o).toFixed(1)}m` : '';

  // Filtered join, not interpolation: a waypoint hit has no `d`, and an empty
  // slot left inline shows up as a run of spaces in the middle of the row.
  return [
    `   ${rank ? ordinal(rank) : '*'}`,
    c.obstacleName,
    o.geometry_type ?? 'circle',
    `center=(${center.x.toFixed(1)}, ${center.y.toFixed(1)})`,
    size,
    `h=${o.height ?? 0}`,
    d,
    `depth_xy=${c.penetrationDepth.xy.toFixed(1)}m depth_z=${c.penetrationDepth.z.toFixed(1)}m`,
  ]
    .filter(Boolean)
    .join('  ');
}

/**
 * Format a route's distance line for the ROUTE DISTANCES section
 * @param {Object} routeResult
 * @returns {string}
 */
function formatRouteDistanceEntry(routeResult) {
  const name = routeResult.routeName || routeResult.uav || `Route ${routeResult.routeId}`;
  return `[ROUTE: ${name}] Distance: ${routeResult.distance.toFixed(1)} m`;
}

/**
 * Format a route's blocked segments.
 *
 * Grouped by segment, with the obstacles ranked in the order the route meets
 * them, because one segment cutting through three obstacles is ONE problem,
 * not three: the planner bypasses the first and lets the rest come back as
 * their own findings. Listed flat, it reads as three independent findings and
 * invites a single waypoint placed to clear all of them at once - which only
 * exists outside the formation, i.e. a perimeter detour.
 * @param {Object} routeResult
 * @returns {string}
 */
function formatRouteCollisionEntry(routeResult) {
  const name = routeResult.routeName || routeResult.uav || `Route ${routeResult.routeId}`;
  if (routeResult.collisions.length === 0) return `[ROUTE: ${name}] clear`;

  const blocks = [];
  const bySegment = new Map();
  const waypointHits = [];

  for (const c of routeResult.collisions) {
    // A waypoint collision has no segment endpoints - a point has no direction
    // along which to be "first", so it cannot join the per-segment ranking.
    if (!c.segmentStart || !c.segmentEnd) {
      waypointHits.push(c);
      continue;
    }
    if (!bySegment.has(c.segmentIndex)) bySegment.set(c.segmentIndex, []);
    bySegment.get(c.segmentIndex).push(c);
  }

  for (const [segmentIndex, hits] of [...bySegment].sort((a, b) => a[0] - b[0])) {
    hits.sort((a, b) => distanceFromStart(a.segmentStart, a.obstacle) - distanceFromStart(b.segmentStart, b.obstacle));
    const { segmentStart: s, segmentEnd: e, label } = hits[0];
    blocks.push(
      [
        `[ROUTE: ${name}] seg ${segmentIndex}  ${label ?? ''}`,
        `   from=(${s.x.toFixed(1)}, ${s.y.toFixed(1)}, ${s.z.toFixed(1)})  ` +
          `to=(${e.x.toFixed(1)}, ${e.y.toFixed(1)}, ${e.z.toFixed(1)})`,
        ...hits.map((c, i) => formatObstacleEntry(c, i + 1)),
      ].join('\n')
    );
  }

  for (const c of waypointHits) {
    blocks.push([`[ROUTE: ${name}] waypoint ${c.segmentIndex}  ${c.label ?? ''}`, formatObstacleEntry(c, null)].join('\n'));
  }

  return blocks.join('\n\n');
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
    `possible collision, possible fixes: generate a detour for one of the UAVs near this point for avoide collision`
  );
}

/**
 * Format full mission validation report
 * @param {Object} missionResult - Result from validateMissionCollission()
 * @returns {string}
 */
export function formatMissionReport(missionResult) {
  const lines = [];
  const blocked = missionResult.routes.filter((r) => r.collisions.length > 0);
  // Count real segments only: a waypoint collision carries the WAYPOINT index
  // in `segmentIndex` (overwritten in validateRoute), so counting it here
  // would inflate the segment tally with indices that name no segment.
  const blockedSegments = new Set(
    blocked.flatMap((r) =>
      r.collisions
        .filter((c) => c.segmentStart && c.segmentEnd)
        .map((c) => `${r.routeName || r.uav || r.routeId}#${c.segmentIndex}`)
    )
  ).size;

  lines.push('--- ROUTE DISTANCES ---');
  lines.push(`Total Distance: ${missionResult.totalDistance.toFixed(1)} m`);
  for (const route of missionResult.routes) {
    lines.push(formatRouteDistanceEntry(route));
  }

  lines.push('');
  lines.push('--- COLLISION OBJECT SEGMENTS ---');
  // Warnings ride along as a subordinate clause on the collision count rather
  // than getting a section of their own. They are caution-zone proximity, not
  // a safety failure, so they must not read as peer findings - but a heading
  // like "NOT ACTIONABLE" reads as a REGION marker, and everything below it
  // (inter-UAV conflicts, inspection coverage) is very much actionable.
  const warned = missionResult.totalWarnings
    ? ` ${missionResult.totalWarnings} caution-zone warning(s) not listed: proximity only, never a collision - no action.`
    : '';
  if (missionResult.totalCollisions === 0) {
    lines.push(`None.${warned}`);
  } else {
    lines.push(`${missionResult.totalCollisions} collision(s) on ${blockedSegments} segment(s).${warned}`);
    lines.push('');
    lines.push(blocked.map(formatRouteCollisionEntry).join('\n\n'));
  }

  lines.push('');
  lines.push('--- INTER-UAV CONFLICTS ---');
  lines.push(`Total: ${missionResult.interRouteCollisions?.length ?? 0}`);
  if (missionResult.interRouteCollisions?.length > 0) {
    for (const c of missionResult.interRouteCollisions) {
      lines.push(formatInterRouteEntry(c));
    }
  }

  return lines.join('\n');
}
