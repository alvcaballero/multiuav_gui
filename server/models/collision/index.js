/**
 * Collision Detection Module
 *
 * Provides algorithmic collision detection and avoidance for UAV missions.
 *
 * @module collision
 *
 * @example
 * import { validateMissionCollission, resolveCollisions } from './collision/index.js';
 *
 * // Validate a mission
 * const validation = validateMissionCollission(mission, obstacles);
 * if (!validation.valid) {
 *   console.log(`Found ${validation.totalCollisions} collisions`);
 * }
 *
 * // Automatically resolve collisions with detours
 * const { mission: safeMission, report } = resolveCollisions(mission, obstacles);
 */

// Collision Detection
export {
  validateRoute,
  validateMissionCollission,
  findCollidingObstacles,
  findInterRouteCollisions,
  formatCollisionReport,
  formatRouteReport,
  formatMissionReport,
} from './collisionDetector.js';

// Detour Generation
export { generateDetour, applyDetoursToRoute, resolveCollisions } from './detourGenerator.js';

// Inspection Coverage Validation
export { validateInspectionCoverage, formatInspectionReport } from './inspectionValidator.js';

// Geometry Utilities (for advanced use)
export {
  distance3D,
  distance2D,
  pointInAABB,
  pointInCylinder,
  pointInOBB,
  segmentIntersectsAABB,
  segmentIntersectsCylinder,
  segmentIntersectsOBB,
  closestPointsBetweenSegments,
  obstacleCenter,
  obstacleCylinder,
  obstacleAABB,
  obstacleOBB,
} from './geometry.js';
