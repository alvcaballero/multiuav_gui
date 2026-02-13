/**
 * Geometry utilities for collision detection
 * Coordinate system: ENU (X+ = East, Y+ = North, Z+ = Up)
 */

/**
 * @typedef {Object} Point3D
 * @property {number} x - East coordinate (meters)
 * @property {number} y - North coordinate (meters)
 * @property {number} z - Up coordinate (meters)
 */

/**
 * @typedef {Object} AABB
 * @property {Point3D} min_point - Minimum corner of bounding box
 * @property {Point3D} max_point - Maximum corner of bounding box
 */

/**
 * @typedef {Object} Cylinder
 * @property {Point3D} center - Base center of cylinder
 * @property {number} radius - Radius in meters
 * @property {number} height - Height in meters (extends from center.z upward)
 */

/**
 * @typedef {Object} Segment
 * @property {Point3D} start - Start point
 * @property {Point3D} end - End point
 */

/**
 * Calculate 3D distance between two points
 * @param {Point3D} p1
 * @param {Point3D} p2
 * @returns {number} Distance in meters
 */
export function distance3D(p1, p2) {
  const dx = p2.x - p1.x;
  const dy = p2.y - p1.y;
  const dz = p2.z - p1.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * Calculate 2D horizontal distance between two points (ignoring Z)
 * @param {Point3D} p1
 * @param {Point3D} p2
 * @returns {number} Distance in meters
 */
export function distance2D(p1, p2) {
  const dx = p2.x - p1.x;
  const dy = p2.y - p1.y;
  return Math.sqrt(dx * dx + dy * dy);
}

/**
 * Check if a point is inside an AABB
 * @param {Point3D} point
 * @param {AABB} aabb
 * @param {number} [margin=0] - Safety margin in meters
 * @returns {boolean}
 */
export function pointInAABB(point, aabb, margin = 0) {
  return (
    point.x >= aabb.min_point.x - margin &&
    point.x <= aabb.max_point.x + margin &&
    point.y >= aabb.min_point.y - margin &&
    point.y <= aabb.max_point.y + margin &&
    point.z >= aabb.min_point.z - margin &&
    point.z <= aabb.max_point.z + margin
  );
}

/**
 * Check if a point is inside a cylinder (vertical axis)
 * @param {Point3D} point
 * @param {Cylinder} cylinder
 * @param {number} [margin=0] - Safety margin in meters
 * @returns {boolean}
 */
export function pointInCylinder(point, cylinder, margin = 0) {
  const horizontalDist = distance2D(point, cylinder.center);
  const inRadius = horizontalDist <= cylinder.radius + margin;
  const inHeight = point.z >= cylinder.center.z - margin &&
                   point.z <= cylinder.center.z + cylinder.height + margin;
  return inRadius && inHeight;
}

/**
 * Check if two AABBs intersect
 * @param {AABB} aabb1
 * @param {AABB} aabb2
 * @param {number} [margin=0] - Safety margin in meters
 * @returns {boolean}
 */
export function aabbIntersectsAABB(aabb1, aabb2, margin = 0) {
  return (
    aabb1.min_point.x - margin <= aabb2.max_point.x + margin &&
    aabb1.max_point.x + margin >= aabb2.min_point.x - margin &&
    aabb1.min_point.y - margin <= aabb2.max_point.y + margin &&
    aabb1.max_point.y + margin >= aabb2.min_point.y - margin &&
    aabb1.min_point.z - margin <= aabb2.max_point.z + margin &&
    aabb1.max_point.z + margin >= aabb2.min_point.z - margin
  );
}

/**
 * Create AABB from a line segment (for segment-AABB intersection tests)
 * @param {Segment} segment
 * @returns {AABB}
 */
export function segmentToAABB(segment) {
  return {
    min_point: {
      x: Math.min(segment.start.x, segment.end.x),
      y: Math.min(segment.start.y, segment.end.y),
      z: Math.min(segment.start.z, segment.end.z),
    },
    max_point: {
      x: Math.max(segment.start.x, segment.end.x),
      y: Math.max(segment.start.y, segment.end.y),
      z: Math.max(segment.start.z, segment.end.z),
    },
  };
}

/**
 * Check if a line segment intersects an AABB using slab method
 * @param {Segment} segment
 * @param {AABB} aabb
 * @param {number} [margin=0] - Safety margin in meters
 * @returns {{intersects: boolean, tMin: number, tMax: number}}
 *          tMin/tMax are parametric values [0,1] along segment
 */
export function segmentIntersectsAABB(segment, aabb, margin = 0) {
  const dir = {
    x: segment.end.x - segment.start.x,
    y: segment.end.y - segment.start.y,
    z: segment.end.z - segment.start.z,
  };

  let tMin = 0;
  let tMax = 1;

  // Expand AABB by margin
  const min = {
    x: aabb.min_point.x - margin,
    y: aabb.min_point.y - margin,
    z: aabb.min_point.z - margin,
  };
  const max = {
    x: aabb.max_point.x + margin,
    y: aabb.max_point.y + margin,
    z: aabb.max_point.z + margin,
  };

  // Check each axis (slab method)
  const axes = ['x', 'y', 'z'];
  for (const axis of axes) {
    if (Math.abs(dir[axis]) < 1e-10) {
      // Ray is parallel to slab
      if (segment.start[axis] < min[axis] || segment.start[axis] > max[axis]) {
        return { intersects: false, tMin: 0, tMax: 0 };
      }
    } else {
      const invD = 1 / dir[axis];
      let t1 = (min[axis] - segment.start[axis]) * invD;
      let t2 = (max[axis] - segment.start[axis]) * invD;

      if (t1 > t2) {
        [t1, t2] = [t2, t1];
      }

      tMin = Math.max(tMin, t1);
      tMax = Math.min(tMax, t2);

      if (tMin > tMax) {
        return { intersects: false, tMin: 0, tMax: 0 };
      }
    }
  }

  return { intersects: true, tMin, tMax };
}

/**
 * Find the closest point on a line segment to a given point
 * @param {Point3D} point
 * @param {Segment} segment
 * @returns {{point: Point3D, t: number}} Closest point and parametric value
 */
export function closestPointOnSegment(point, segment) {
  const dx = segment.end.x - segment.start.x;
  const dy = segment.end.y - segment.start.y;
  const dz = segment.end.z - segment.start.z;

  const lengthSq = dx * dx + dy * dy + dz * dz;

  if (lengthSq < 1e-10) {
    // Segment is a point
    return { point: { ...segment.start }, t: 0 };
  }

  // Parametric value along segment
  let t = (
    (point.x - segment.start.x) * dx +
    (point.y - segment.start.y) * dy +
    (point.z - segment.start.z) * dz
  ) / lengthSq;

  // Clamp to [0, 1]
  t = Math.max(0, Math.min(1, t));

  return {
    point: {
      x: segment.start.x + t * dx,
      y: segment.start.y + t * dy,
      z: segment.start.z + t * dz,
    },
    t,
  };
}

/**
 * Check if a line segment intersects a cylinder (vertical axis)
 * Uses closest point approach for robustness
 * @param {Segment} segment
 * @param {Cylinder} cylinder
 * @param {number} [margin=0] - Safety margin in meters
 * @returns {{intersects: boolean, closestPoint: Point3D, distance: number}}
 */
export function segmentIntersectsCylinder(segment, cylinder, margin = 0) {
  // Project to 2D (XY plane) for horizontal distance check
  const segment2D = {
    start: { x: segment.start.x, y: segment.start.y, z: 0 },
    end: { x: segment.end.x, y: segment.end.y, z: 0 },
  };
  const center2D = { x: cylinder.center.x, y: cylinder.center.y, z: 0 };

  // Find closest point on 2D segment to cylinder center
  const closest2D = closestPointOnSegment(center2D, segment2D);
  const horizontalDist = distance2D(closest2D.point, center2D);

  // Get the 3D point at that parametric value
  const point3D = {
    x: segment.start.x + closest2D.t * (segment.end.x - segment.start.x),
    y: segment.start.y + closest2D.t * (segment.end.y - segment.start.y),
    z: segment.start.z + closest2D.t * (segment.end.z - segment.start.z),
  };

  // Check if within cylinder bounds
  const inRadius = horizontalDist <= cylinder.radius + margin;
  const inHeight = point3D.z >= cylinder.center.z - margin &&
                   point3D.z <= cylinder.center.z + cylinder.height + margin;

  // Also check segment endpoints for cases where segment passes through
  const startInCylinder = pointInCylinder(segment.start, cylinder, margin);
  const endInCylinder = pointInCylinder(segment.end, cylinder, margin);

  const intersects = (inRadius && inHeight) || startInCylinder || endInCylinder;

  return {
    intersects,
    closestPoint: point3D,
    distance: horizontalDist,
  };
}

/**
 * Interpolate a point along a segment
 * @param {Segment} segment
 * @param {number} t - Parametric value [0, 1]
 * @returns {Point3D}
 */
export function interpolateSegment(segment, t) {
  return {
    x: segment.start.x + t * (segment.end.x - segment.start.x),
    y: segment.start.y + t * (segment.end.y - segment.start.y),
    z: segment.start.z + t * (segment.end.z - segment.start.z),
  };
}

/**
 * Create a cylinder from obstacle data with zone information
 * Parses zone strings like:
 *   - Spanish: "Cilindro: radio 35m, altura 108m (z: 0→108)."
 *   - English: "cylinder: radius=30m, height=108m"
 * @param {Object} obstacle - Obstacle with position and zones
 * @param {string} zoneType - 'exclusion_zone' | 'caution_zone'
 * @returns {Cylinder|null}
 */
export function cylinderFromObstacleZone(obstacle, zoneType = 'exclusion_zone') {
  const zone = obstacle.zones?.[zoneType];
  if (!zone) return null;

  // Parse zone string for radius and height (supports Spanish and English formats)
  // Spanish: "radio 35m" / English: "radius=30m" or "radius 30m"
  const radiusMatch = zone.match(/(?:radio|radius)[=:\s]+(\d+(?:\.\d+)?)\s*m/i);
  // Spanish: "altura 108m" / English: "height=108m" or "height 108m"
  const heightMatch = zone.match(/(?:altura|height)[=:\s]+(\d+(?:\.\d+)?)\s*m/i);

  if (!radiusMatch || !heightMatch) {
    // Fallback to AABB dimensions if zone parsing fails
    if (obstacle.aabb) {
      const aabb = obstacle.aabb;
      const radiusX = (aabb.max_point.x - aabb.min_point.x) / 2;
      const radiusY = (aabb.max_point.y - aabb.min_point.y) / 2;
      return {
        center: obstacle.position,
        radius: Math.max(radiusX, radiusY),
        height: aabb.max_point.z - aabb.min_point.z,
      };
    }
    return null;
  }

  return {
    center: { ...obstacle.position },
    radius: parseFloat(radiusMatch[1]),
    height: parseFloat(heightMatch[1]),
  };
}

/**
 * Calculate segment length
 * @param {Segment} segment
 * @returns {number}
 */
export function segmentLength(segment) {
  return distance3D(segment.start, segment.end);
}

/**
 * Normalize a 3D vector
 * @param {Point3D} v
 * @returns {Point3D}
 */
export function normalize(v) {
  const len = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  if (len < 1e-10) return { x: 0, y: 0, z: 0 };
  return { x: v.x / len, y: v.y / len, z: v.z / len };
}

/**
 * Calculate perpendicular vector in XY plane (90 degrees counterclockwise)
 * @param {Point3D} v - Direction vector
 * @returns {Point3D} Perpendicular vector (same length, Z=0)
 */
export function perpendicular2D(v) {
  return { x: -v.y, y: v.x, z: 0 };
}
