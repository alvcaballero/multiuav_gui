/**
 * Collision Detection Module
 *
 * Provides algorithmic collision detection and avoidance for UAV missions.
 *
 * @module collision
 *
 * @example
 * import { validateMission, resolveCollisions } from './collision/index.js';
 *
 * // Validate a mission
 * const validation = validateMission(mission, obstacles);
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
  validateMission,
  findCollidingObstacles,
  formatCollisionReport,
  formatRouteReport,
  formatMissionReport,
} from './collisionDetector.js';

// Detour Generation
export {
  generateDetour,
  applyDetoursToRoute,
  resolveCollisions,
} from './detourGenerator.js';

// Geometry Utilities (for advanced use)
export {
  distance3D,
  distance2D,
  pointInAABB,
  pointInCylinder,
  segmentIntersectsAABB,
  segmentIntersectsCylinder,
  cylinderFromObstacleZone,
} from './geometry.js';
