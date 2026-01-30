/**
 * Detour Generator for UAV Mission Routes
 * Generates alternative waypoints to avoid obstacles
 */

import logger from '../../common/logger.js';
import {
  distance2D,
  distance3D,
  normalize,
  perpendicular2D,
  segmentIntersectsCylinder,
  cylinderFromObstacleZone,
  interpolateSegment,
} from './geometry.js';
import { validateRoute, findCollidingObstacles } from './collisionDetector.js';

/**
 * @typedef {import('./geometry.js').Point3D} Point3D
 * @typedef {import('./collisionDetector.js').Obstacle} Obstacle
 * @typedef {import('./collisionDetector.js').Waypoint} Waypoint
 * @typedef {import('./collisionDetector.js').CollisionResult} CollisionResult
 */

/**
 * @typedef {Object} DetourStrategy
 * @property {'lateral'|'vertical'|'combined'} type - Detour type
 * @property {'north'|'south'|'east'|'west'|'left'|'right'|'over'} direction - Direction
 * @property {number} clearance - Distance from obstacle (meters)
 */

// Configuration
const CONFIG = {
  MIN_CLEARANCE: 10,        // Minimum clearance from obstacles (meters)
  DEFAULT_CLEARANCE: 15,    // Default clearance
  MAX_ALTITUDE: 120,        // Maximum flight altitude (meters)
  ALTITUDE_BUFFER: 10,      // Buffer above obstacles for vertical detours
  WAYPOINT_SPACING: 20,     // Minimum spacing between generated waypoints
  MAX_DETOUR_WAYPOINTS: 4,  // Maximum waypoints per detour
};

/**
 * Normalize waypoint position to Point3D
 * @param {Point3D|number[]} pos
 * @returns {Point3D}
 */
function normalizePos(pos) {
  if (Array.isArray(pos)) {
    return { x: pos[0], y: pos[1], z: pos[2] };
  }
  return pos;
}

/**
 * Convert Point3D to array format
 * @param {Point3D} pos
 * @returns {number[]}
 */
function posToArray(pos) {
  return [pos.x, pos.y, pos.z];
}

/**
 * Determine best detour direction based on obstacle position relative to flight path
 * @param {Point3D} start - Segment start
 * @param {Point3D} end - Segment end
 * @param {Obstacle} obstacle
 * @returns {DetourStrategy}
 */
function determineBestDetourStrategy(start, end, obstacle) {
  const obstaclePos = obstacle.position;

  // Calculate flight direction vector
  const flightDir = normalize({
    x: end.x - start.x,
    y: end.y - start.y,
    z: end.z - start.z,
  });

  // Get perpendicular (left side of flight path)
  const perpLeft = perpendicular2D(flightDir);

  // Vector from start to obstacle
  const toObstacle = {
    x: obstaclePos.x - start.x,
    y: obstaclePos.y - start.y,
    z: obstaclePos.z - start.z,
  };

  // Dot product with perpendicular to determine which side obstacle is on
  const crossProduct = toObstacle.x * perpLeft.x + toObstacle.y * perpLeft.y;

  // Get obstacle height from exclusion zone
  const cylinder = cylinderFromObstacleZone(obstacle, 'exclusion_zone');
  const obstacleHeight = cylinder ? cylinder.height : 100;
  const obstacleRadius = cylinder ? cylinder.radius : 30;

  // Determine clearance based on caution zone
  const cautionCylinder = cylinderFromObstacleZone(obstacle, 'caution_zone');
  const clearance = cautionCylinder
    ? cautionCylinder.radius + CONFIG.MIN_CLEARANCE
    : obstacleRadius + CONFIG.DEFAULT_CLEARANCE;

  // Check if vertical detour is viable
  const canGoOver = obstacleHeight + CONFIG.ALTITUDE_BUFFER <= CONFIG.MAX_ALTITUDE;
  const currentAltitude = Math.max(start.z, end.z);

  // Prefer lateral if possible (more energy efficient)
  // Go to the opposite side of where the obstacle is
  if (crossProduct > 0) {
    // Obstacle is on the left, go right
    return { type: 'lateral', direction: 'right', clearance };
  } else if (crossProduct < 0) {
    // Obstacle is on the right, go left
    return { type: 'lateral', direction: 'left', clearance };
  } else {
    // Obstacle is directly ahead
    if (canGoOver && obstacleHeight < 50) {
      // Prefer going over for shorter obstacles
      return { type: 'vertical', direction: 'over', clearance: CONFIG.ALTITUDE_BUFFER };
    } else {
      // Default to right side
      return { type: 'lateral', direction: 'right', clearance };
    }
  }
}

/**
 * Generate lateral detour waypoints
 * @param {Point3D} start
 * @param {Point3D} end
 * @param {Obstacle} obstacle
 * @param {DetourStrategy} strategy
 * @returns {Point3D[]}
 */
function generateLateralDetour(start, end, obstacle, strategy) {
  const waypoints = [];
  const obstaclePos = obstacle.position;

  // Flight direction
  const flightDir = normalize({
    x: end.x - start.x,
    y: end.y - start.y,
    z: 0,
  });

  // Perpendicular direction (left or right)
  let perpDir = perpendicular2D(flightDir);
  if (strategy.direction === 'right') {
    perpDir = { x: -perpDir.x, y: -perpDir.y, z: 0 };
  }

  // Get cylinder for accurate dimensions
  const cylinder = cylinderFromObstacleZone(obstacle, 'caution_zone') ||
                   cylinderFromObstacleZone(obstacle, 'exclusion_zone');
  const radius = cylinder ? cylinder.radius : 30;

  // Calculate offset distance
  const offsetDist = radius + strategy.clearance;

  // Entry point: before the obstacle
  const distToObstacle = distance2D(start, obstaclePos);
  const approachDist = Math.max(offsetDist, distToObstacle * 0.3);

  // Find point closest to obstacle on flight path
  const t = Math.max(0, Math.min(1,
    ((obstaclePos.x - start.x) * flightDir.x + (obstaclePos.y - start.y) * flightDir.y) /
    (distance2D(start, end) || 1)
  ));

  const closestPoint = interpolateSegment({ start, end }, t);

  // Entry waypoint (before obstacle, offset to the side)
  const entryT = Math.max(0.1, t - 0.2);
  const entryBase = interpolateSegment({ start, end }, entryT);
  waypoints.push({
    x: entryBase.x + perpDir.x * offsetDist,
    y: entryBase.y + perpDir.y * offsetDist,
    z: Math.max(start.z, end.z), // Maintain altitude
  });

  // Middle waypoint (at obstacle level, fully offset)
  waypoints.push({
    x: closestPoint.x + perpDir.x * offsetDist,
    y: closestPoint.y + perpDir.y * offsetDist,
    z: Math.max(start.z, end.z),
  });

  // Exit waypoint (after obstacle, back towards path)
  const exitT = Math.min(0.9, t + 0.2);
  const exitBase = interpolateSegment({ start, end }, exitT);
  waypoints.push({
    x: exitBase.x + perpDir.x * offsetDist,
    y: exitBase.y + perpDir.y * offsetDist,
    z: Math.max(start.z, end.z),
  });

  return waypoints;
}

/**
 * Generate vertical (over) detour waypoints
 * @param {Point3D} start
 * @param {Point3D} end
 * @param {Obstacle} obstacle
 * @param {DetourStrategy} strategy
 * @returns {Point3D[]}
 */
function generateVerticalDetour(start, end, obstacle, strategy) {
  const waypoints = [];
  const obstaclePos = obstacle.position;

  // Get obstacle height
  const cylinder = cylinderFromObstacleZone(obstacle, 'exclusion_zone');
  const obstacleHeight = cylinder ? cylinder.height : 100;
  const safeAltitude = Math.min(
    obstacleHeight + strategy.clearance,
    CONFIG.MAX_ALTITUDE
  );

  // Flight direction
  const flightDir = normalize({
    x: end.x - start.x,
    y: end.y - start.y,
    z: 0,
  });

  // Find closest point to obstacle on path
  const t = Math.max(0, Math.min(1,
    ((obstaclePos.x - start.x) * flightDir.x + (obstaclePos.y - start.y) * flightDir.y) /
    (distance2D(start, end) || 1)
  ));

  // Climb waypoint (before obstacle)
  const climbT = Math.max(0.1, t - 0.15);
  const climbBase = interpolateSegment({ start, end }, climbT);
  waypoints.push({
    x: climbBase.x,
    y: climbBase.y,
    z: safeAltitude,
  });

  // Over waypoint (directly above obstacle area)
  const overPoint = interpolateSegment({ start, end }, t);
  waypoints.push({
    x: overPoint.x,
    y: overPoint.y,
    z: safeAltitude,
  });

  // Descent waypoint (after obstacle)
  const descentT = Math.min(0.9, t + 0.15);
  const descentBase = interpolateSegment({ start, end }, descentT);
  waypoints.push({
    x: descentBase.x,
    y: descentBase.y,
    z: safeAltitude,
  });

  return waypoints;
}

/**
 * Generate detour waypoints for a single collision
 * @param {Point3D} start - Segment start
 * @param {Point3D} end - Segment end
 * @param {Obstacle} obstacle
 * @returns {{waypoints: Point3D[], strategy: DetourStrategy}}
 */
export function generateDetour(start, end, obstacle) {
  const strategy = determineBestDetourStrategy(start, end, obstacle);

  let waypoints;
  if (strategy.type === 'vertical') {
    waypoints = generateVerticalDetour(start, end, obstacle, strategy);
  } else {
    waypoints = generateLateralDetour(start, end, obstacle, strategy);
  }

  logger.info(
    `[DetourGenerator] Generated ${strategy.type} detour (${strategy.direction}) ` +
    `around ${obstacle.name} with ${waypoints.length} waypoints`
  );

  return { waypoints, strategy };
}

/**
 * Apply detours to a route with collisions
 * @param {Waypoint[]} waypoints - Original waypoints
 * @param {CollisionResult[]} collisions - Detected collisions
 * @param {Obstacle[]} obstacles - All obstacles for validation
 * @returns {{waypoints: Waypoint[], detoursApplied: number, report: string[]}}
 */
export function applyDetoursToRoute(waypoints, collisions, obstacles) {
  if (collisions.length === 0) {
    return { waypoints, detoursApplied: 0, report: ['No collisions to resolve'] };
  }

  const report = [];
  let detoursApplied = 0;

  // Group collisions by segment
  const collisionsBySegment = new Map();
  for (const collision of collisions) {
    const key = collision.segmentIndex;
    if (!collisionsBySegment.has(key)) {
      collisionsBySegment.set(key, []);
    }
    collisionsBySegment.get(key).push(collision);
  }

  // Process segments in reverse order to maintain indices
  const sortedSegments = Array.from(collisionsBySegment.keys()).sort((a, b) => b - a);

  const newWaypoints = [...waypoints];

  for (const segmentIndex of sortedSegments) {
    const segmentCollisions = collisionsBySegment.get(segmentIndex);
    if (segmentIndex < 0 || segmentIndex >= newWaypoints.length - 1) continue;

    const start = normalizePos(newWaypoints[segmentIndex].pos);
    const end = normalizePos(newWaypoints[segmentIndex + 1].pos);

    // Handle the primary (first) collision for this segment
    const primaryCollision = segmentCollisions[0];
    const { waypoints: detourPoints, strategy } = generateDetour(
      start,
      end,
      primaryCollision.obstacle
    );

    // Validate detour doesn't create new collisions
    const detourWps = detourPoints.map((p, i) => ({
      type: 'transit',
      pos: p,
      notes: `Detour ${strategy.type} around ${primaryCollision.obstacleName} (${i + 1}/${detourPoints.length})`,
    }));

    // Add start and end for validation
    const fullPath = [
      newWaypoints[segmentIndex],
      ...detourWps,
      newWaypoints[segmentIndex + 1],
    ];

    const validation = validateRoute(fullPath, obstacles);

    if (validation.valid) {
      // Insert detour waypoints
      newWaypoints.splice(segmentIndex + 1, 0, ...detourWps);
      detoursApplied++;
      report.push(
        `Segment ${segmentIndex}: Applied ${strategy.type} detour (${strategy.direction}) ` +
        `around ${primaryCollision.obstacleName}, added ${detourPoints.length} waypoints`
      );
    } else {
      // Try alternative strategy
      report.push(
        `Segment ${segmentIndex}: Primary detour invalid, trying alternative...`
      );

      // Try vertical if lateral failed, or vice versa
      const altStrategy = strategy.type === 'lateral'
        ? { type: 'vertical', direction: 'over', clearance: CONFIG.ALTITUDE_BUFFER }
        : { type: 'lateral', direction: strategy.direction === 'left' ? 'right' : 'left', clearance: strategy.clearance };

      let altWaypoints;
      if (altStrategy.type === 'vertical') {
        altWaypoints = generateVerticalDetour(start, end, primaryCollision.obstacle, altStrategy);
      } else {
        altWaypoints = generateLateralDetour(start, end, primaryCollision.obstacle, altStrategy);
      }

      const altDetourWps = altWaypoints.map((p, i) => ({
        type: 'transit',
        pos: p,
        notes: `Detour ${altStrategy.type} around ${primaryCollision.obstacleName} (${i + 1}/${altWaypoints.length})`,
      }));

      const altFullPath = [
        newWaypoints[segmentIndex],
        ...altDetourWps,
        newWaypoints[segmentIndex + 1],
      ];

      const altValidation = validateRoute(altFullPath, obstacles);

      if (altValidation.valid) {
        newWaypoints.splice(segmentIndex + 1, 0, ...altDetourWps);
        detoursApplied++;
        report.push(
          `Segment ${segmentIndex}: Applied alternative ${altStrategy.type} detour ` +
          `around ${primaryCollision.obstacleName}`
        );
      } else {
        report.push(
          `Segment ${segmentIndex}: WARNING - Could not find valid detour around ` +
          `${primaryCollision.obstacleName}. Manual intervention required.`
        );
      }
    }
  }

  return { waypoints: newWaypoints, detoursApplied, report };
}

/**
 * Resolve all collisions in a mission
 * @param {Object} mission - Mission with routes
 * @param {Obstacle[]} obstacles
 * @returns {Object} Modified mission with detours applied
 */
export function resolveCollisions(mission, obstacles) {
  if (!mission?.route || !Array.isArray(mission.route)) {
    return { mission, totalDetoursApplied: 0, report: ['No routes in mission'] };
  }

  const modifiedMission = JSON.parse(JSON.stringify(mission));
  let totalDetoursApplied = 0;
  const report = [];

  for (let i = 0; i < modifiedMission.route.length; i++) {
    const route = modifiedMission.route[i];
    report.push(`\n=== Route ${route.id}: ${route.name} (${route.uav}) ===`);

    // Validate route
    const validation = validateRoute(route.wp || [], obstacles);

    if (validation.valid) {
      report.push('No collisions detected');
      continue;
    }

    report.push(`Found ${validation.collisions.length} collisions, ${validation.warnings.length} warnings`);

    // Apply detours
    const result = applyDetoursToRoute(route.wp, validation.collisions, obstacles);

    modifiedMission.route[i].wp = result.waypoints;
    totalDetoursApplied += result.detoursApplied;
    report.push(...result.report);

    // Final validation
    const finalValidation = validateRoute(result.waypoints, obstacles);
    if (finalValidation.valid) {
      report.push(`Route ${route.id}: All collisions resolved successfully`);
    } else {
      report.push(
        `Route ${route.id}: WARNING - ${finalValidation.collisions.length} collisions remain`
      );
    }
  }

  logger.info(
    `[DetourGenerator] Mission collision resolution complete: ` +
    `${totalDetoursApplied} detours applied`
  );

  return {
    mission: modifiedMission,
    totalDetoursApplied,
    report,
  };
}
