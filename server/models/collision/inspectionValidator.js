/**
 * Inspection Coverage Validation for UAV Mission Routes
 * Verifies that every required target was actually inspected by some
 * waypoint of the mission (close enough AND pointed at it), instead of
 * trusting the LLM's own claim that it inspected a target.
 */

import { logger } from '../../common/logger.js';
import { distance3D } from './geometry.js';

/**
 * @typedef {import('./geometry.js').Point3D} Point3D
 * @typedef {import('../markers/inspectionTargets.js').InspectionTarget} InspectionTarget
 */

const INSPECTION_DEFAULTS = {
  RADIUS: 50, // meters - waypoint must be within this distance of the target
  YAW_TOLERANCE: 30, // degrees - waypoint yaw must point at the target within this tolerance
};

function normalizePosition(pos) {
  if (Array.isArray(pos)) {
    return { x: pos[0], y: pos[1], z: pos[2] };
  }
  return pos;
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
  const dist = distance3D(pos, target.position);

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
 * waypoint across all routes of the mission.
 * @param {Object} mission - Mission object with route[].wp[]
 * @param {InspectionTarget[]} targets - Targets resolved from the catalog (never LLM-supplied positions)
 * @param {Object} [options]
 * @param {number} [options.radius] - Meters; default INSPECTION_DEFAULTS.RADIUS
 * @param {number} [options.yawTolerance] - Degrees; default INSPECTION_DEFAULTS.YAW_TOLERANCE
 * @returns {{valid: boolean, covered: Object[], missing: Object[]}}
 */
export function validateInspectionCoverage(mission, targets, options = {}) {
  const radius = options.radius ?? INSPECTION_DEFAULTS.RADIUS;
  const yawTolerance = options.yawTolerance ?? INSPECTION_DEFAULTS.YAW_TOLERANCE;

  const covered = [];
  const missing = [];

  const routes = mission?.route ?? [];

  for (const target of targets) {
    let bestMatch = null;

    for (const route of routes) {
      const wp = route.wp ?? [];
      for (let i = 0; i < wp.length; i++) {
        const result = checkWaypointCoverage(wp[i], target, radius, yawTolerance);
        if (result.covered && (!bestMatch || result.distance < bestMatch.distance)) {
          bestMatch = {
            targetId: target.id,
            targetName: target.name,
            routeId: route.id,
            routeName: route.name,
            uav: route.uav,
            waypointIndex: i,
            distance: result.distance,
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
  const status = result.valid ? 'VALID (All targets inspected)' : 'INCOMPLETE (Missing targets)';

  lines.push(`Status: ${status}`);
  lines.push(`**Covered:** ${result.covered.length} | **Missing:** ${result.missing.length}`);

  if (result.missing.length > 0) {
    lines.push('');
    lines.push('#### [MISSING INSPECTIONS]');
    for (const m of result.missing) {
      const point = `(${m.position.x.toFixed(1)}, ${m.position.y.toFixed(1)}, ${m.position.z.toFixed(1)})`;
      lines.push(
        `  * You forgot to inspect **${m.targetName}** (id=${m.targetId}, type=${m.type ?? 'unknown'})` +
          ` at position ${point}. No waypoint points at it - add an inspection waypoint there.`
      );
    }
  }

  return lines.join('\n');
}
