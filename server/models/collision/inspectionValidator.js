/**
 * Inspection Coverage Validation for UAV Mission Routes
 * Verifies that every required target was actually inspected by some
 * waypoint of the mission (close enough AND pointed at it), instead of
 * trusting the LLM's own claim that it inspected a target.
 */

import { logger } from '../../common/logger.js';
import { distance2D } from './geometry.js';

/**
 * @typedef {import('./geometry.js').Point3D} Point3D
 * @typedef {import('../markers/inspectionTargets.js').InspectionTarget} InspectionTarget
 */

const INSPECTION_DEFAULTS = {
  RADIUS: 50, // meters - fallback/base margin when no obstacle geometry is available
  YAW_TOLERANCE: 30, // degrees - waypoint yaw must point at the target within this tolerance
  SIZE_MARGIN_FACTOR: 1.2, // multiplier applied to the obstacle's own footprint
};

function normalizePosition(pos) {
  if (Array.isArray(pos)) {
    return { x: pos[0], y: pos[1], z: pos[2] };
  }
  return pos;
}

/**
 * Horizontal footprint radius of an LLM-declared obstacle (from
 * collision_objects), regardless of geometry_type: the circle radius, or
 * half the longer side of a rectangle (conservative - covers rotation).
 * @param {Object} obstacle - Entry from collision_objects
 * @returns {number}
 */
function obstacleHorizontalExtent(obstacle) {
  if (obstacle.geometry_type === 'rectangle') {
    const width = obstacle.dimensions?.width ?? 0;
    const length = obstacle.dimensions?.length ?? 0;
    return Math.max(width, length) / 2;
  }
  return obstacle.dimensions?.radius ?? 0;
}

/**
 * Find the collision_objects entry that represents a given inspection
 * target, matching by id or name (both are LLM-declared, so cross-check
 * both instead of trusting either alone). Falls back to the geometrically
 * closest obstacle when no direct match is found - the LLM may have
 * described the same physical object under a different id/name in
 * collision_objects.
 * @param {InspectionTarget} target
 * @param {Object[]} collisionObjects - Entries from the request's collision_objects
 * @returns {Object|null}
 */
function findMatchingObstacle(target, collisionObjects) {
  if (!Array.isArray(collisionObjects) || collisionObjects.length === 0) {
    return null;
  }

  const targetIdStr = String(target.id);
  const direct = collisionObjects.find(
    (obstacle) => obstacle.obstacle_id === targetIdStr || obstacle.obstacle_name === target.name
  );
  if (direct) return direct;

  let closest = null;
  let closestDist = Infinity;
  for (const obstacle of collisionObjects) {
    if (!obstacle.position) continue;
    const dist = distance2D(target.position, obstacle.position);
    if (dist < closestDist) {
      closestDist = dist;
      closest = obstacle;
    }
  }
  return closest;
}

/**
 * Effective inspection radius for a target: derived from the size of the
 * physical object it represents (as declared by the LLM in
 * collision_objects), not a single global constant - a 56m-radius wind
 * turbine needs a much larger coverage radius than a 1.5m power tower.
 * @param {InspectionTarget} target
 * @param {Object[]} collisionObjects
 * @param {number} baseRadius - Fallback/base margin (INSPECTION_DEFAULTS.RADIUS)
 * @param {number} sizeMarginFactor
 * @returns {number}
 */
function resolveEffectiveRadius(target, collisionObjects, baseRadius, sizeMarginFactor) {
  const obstacle = findMatchingObstacle(target, collisionObjects);
  if (!obstacle) return baseRadius;

  const extent = obstacleHorizontalExtent(obstacle);
  return extent * sizeMarginFactor + baseRadius;
}

/**
 * Bearing from `from` to `to` in the XY plane, degrees, 0=North(+Y)/90=East(+X),
 * matching the mission waypoint yaw convention (see server/CLAUDE.md).
 * @param {Point3D} from
 * @param {Point3D} to
 * @returns {number}
 */
function bearingDegrees(from, to) {
  const dx = to.x - from.x;
  const dy = to.y - from.y;
  return (Math.atan2(dx, dy) * 180) / Math.PI;
}

function angleDiffDegrees(a, b) {
  let diff = Math.abs(a - b) % 360;
  if (diff > 180) diff = 360 - diff;
  return diff;
}

/**
 * Check whether a single waypoint counts as inspecting a target: within
 * radius AND yaw pointed at it within tolerance. Waypoints without a yaw
 * can't satisfy the pointing requirement (distance alone isn't inspection).
 * @param {Object} waypoint
 * @param {InspectionTarget} target
 * @param {number} radius
 * @param {number} yawTolerance
 * @returns {{covered: boolean, distance: number}}
 */
function checkWaypointCoverage(waypoint, target, radius, yawTolerance) {
  const pos = normalizePosition(waypoint.pos);
  const dist = distance2D(pos, target.position);

  if (dist > radius) {
    return { covered: false, distance: dist };
  }
  if (typeof waypoint.yaw !== 'number') {
    return { covered: false, distance: dist };
  }

  const bearing = bearingDegrees(pos, target.position);
  const covered = angleDiffDegrees(waypoint.yaw, bearing) <= yawTolerance;
  return { covered, distance: dist };
}

/**
 * Validate that every required inspection target is covered by at least one
 * waypoint across all routes of the mission. The coverage radius is derived
 * per-target from the object's own footprint (declared by the LLM in
 * collision_objects) instead of a single fixed distance, since a wind
 * turbine and a power tower need very different inspection radii.
 * @param {Object} mission - Mission object with route[].wp[]
 * @param {InspectionTarget[]} targets - Targets resolved from the catalog (never LLM-supplied positions)
 * @param {Object[]} [collisionObjects] - The request's collision_objects, used to size the coverage radius per target
 * @param {Object} [options]
 * @param {number} [options.radius] - Meters; base/fallback margin, default INSPECTION_DEFAULTS.RADIUS
 * @param {number} [options.yawTolerance] - Degrees; default INSPECTION_DEFAULTS.YAW_TOLERANCE
 * @param {number} [options.sizeMarginFactor] - Multiplier on the obstacle's footprint; default INSPECTION_DEFAULTS.SIZE_MARGIN_FACTOR
 * @returns {{valid: boolean, covered: Object[], missing: Object[]}}
 */
export function validateInspectionCoverage(mission, targets, collisionObjects = [], options = {}) {
  const baseRadius = options.radius ?? INSPECTION_DEFAULTS.RADIUS;
  const yawTolerance = options.yawTolerance ?? INSPECTION_DEFAULTS.YAW_TOLERANCE;
  const sizeMarginFactor = options.sizeMarginFactor ?? INSPECTION_DEFAULTS.SIZE_MARGIN_FACTOR;

  const covered = [];
  const missing = [];

  const routes = mission?.route ?? [];

  for (const target of targets) {
    const radius = resolveEffectiveRadius(target, collisionObjects, baseRadius, sizeMarginFactor);
    let bestMatch = null;

    for (const route of routes) {
      const wp = route.wp ?? [];
      for (let i = 0; i < wp.length; i++) {
        const result = checkWaypointCoverage(wp[i], target, radius, yawTolerance);
        console.log(
          `[InspectionValidator] Checking target ${target.name} (id=${target.id}) against route ${route.name} waypoint ${i}: ` +
            `distance=${result.distance.toFixed(1)}m, radius=${radius.toFixed(1)}m, covered=${result.covered}`
        );
        if (result.covered && (!bestMatch || result.distance < bestMatch.distance)) {
          bestMatch = {
            targetId: target.id,
            targetName: target.name,
            routeId: route.id,
            routeName: route.name,
            uav: route.uav,
            waypointIndex: i,
            distance: result.distance,
            radius,
          };
        }
      }
    }

    if (bestMatch) {
      covered.push(bestMatch);
    } else {
      missing.push({
        targetId: target.id,
        targetName: target.name,
        type: target.type,
        groupName: target.groupName,
        position: target.position,
        radius,
      });
    }
  }

  const result = { valid: missing.length === 0, covered, missing };

  logger.info(
    `[InspectionValidator] Coverage check: valid=${result.valid}, ` +
      `covered=${covered.length}/${targets.length}, missing=${missing.length}`
  );

  return result;
}

/**
 * Format a human/LLM-readable inspection coverage report.
 * @param {Object} result - Result from validateInspectionCoverage()
 * @returns {string}
 */
export function formatInspectionReport(result) {
  const lines = [];

  lines.push('--- INSPECTION COVERAGE ---');
  lines.push(`Covered: ${result.covered.length} | Missing: ${result.missing.length}`);

  if (result.missing.length > 0) {
    lines.push('');
    lines.push('#### [MISSING INSPECTIONS]');
    for (const m of result.missing) {
      const point = `(${m.position.x.toFixed(1)}, ${m.position.y.toFixed(1)}, ${m.position.z.toFixed(1)})`;
      lines.push(
        `  * You forgot to inspect **${m.targetName}** (id=${m.targetId}, type=${m.type ?? 'unknown'})` +
          ` at position ${point}. No waypoint within ${m.radius.toFixed(1)}m points at it - add an inspection waypoint there.`
      );
    }
  }

  return lines.join('\n');
}
