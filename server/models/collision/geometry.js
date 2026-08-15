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
  const inHeight = point.z >= cylinder.center.z - margin && point.z <= cylinder.center.z + cylinder.height + margin;
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
  let t =
    ((point.x - segment.start.x) * dx + (point.y - segment.start.y) * dy + (point.z - segment.start.z) * dz) / lengthSq;

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
  const inHeight = point3D.z >= cylinder.center.z - margin && point3D.z <= cylinder.center.z + cylinder.height + margin;

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
 * Find the closest points between two line segments in 3D (Ericson,
 * "Real-Time Collision Detection", ClosestPtSegmentSegment).
 * @param {Segment} seg1
 * @param {Segment} seg2
 * @returns {{pointA: Point3D, pointB: Point3D, tA: number, tB: number, distance: number}}
 *          tA/tB are parametric values [0,1] along seg1/seg2 at closest approach
 */
export function closestPointsBetweenSegments(seg1, seg2) {
  const EPS = 1e-10;
  const p1 = seg1.start;
  const p2 = seg2.start;
  const d1 = { x: seg1.end.x - p1.x, y: seg1.end.y - p1.y, z: seg1.end.z - p1.z };
  const d2 = { x: seg2.end.x - p2.x, y: seg2.end.y - p2.y, z: seg2.end.z - p2.z };
  const r = { x: p1.x - p2.x, y: p1.y - p2.y, z: p1.z - p2.z };

  const dot = (u, v) => u.x * v.x + u.y * v.y + u.z * v.z;

  const a = dot(d1, d1);
  const e = dot(d2, d2);
  const f = dot(d2, r);

  let s, t;

  if (a <= EPS && e <= EPS) {
    s = 0;
    t = 0;
  } else if (a <= EPS) {
    s = 0;
    t = Math.max(0, Math.min(1, f / e));
  } else {
    const c = dot(d1, r);
    if (e <= EPS) {
      t = 0;
      s = Math.max(0, Math.min(1, -c / a));
    } else {
      const b = dot(d1, d2);
      const denom = a * e - b * b;
      s = denom !== 0 ? Math.max(0, Math.min(1, (b * f - c * e) / denom)) : 0;
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = Math.max(0, Math.min(1, -c / a));
      } else if (t > 1) {
        t = 1;
        s = Math.max(0, Math.min(1, (b - c) / a));
      }
    }
  }

  const pointA = { x: p1.x + d1.x * s, y: p1.y + d1.y * s, z: p1.z + d1.z * s };
  const pointB = { x: p2.x + d2.x * t, y: p2.y + d2.y * t, z: p2.z + d2.z * t };

  return { pointA, pointB, tA: s, tB: t, distance: distance3D(pointA, pointB) };
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
 * @typedef {Object} Obstacle
 * @property {string} obstacle_id - Obstacle identifier
 * @property {string} [obstacle_name] - Human-readable obstacle name, used in reports
 * @property {'circle'|'rectangle'} geometry_type - Geometry used to model the obstacle
 * @property {Point3D} position - Horizontal center (x, y) and base/ground level (z) of the obstacle. z is NOT the geometric centroid - height extends upward from z.
 * @property {{radius:number}|{width:number, length:number}} dimensions - Real footprint size, before rotation by yaw and before safety_margin. radius (geometry_type: 'circle') or width (local West-East extent at yaw=0) + length (local North-South extent at yaw=0) (geometry_type: 'rectangle')
 * @property {number} safety_margin - Extra clearance in meters to add around the obstacle's real geometry. Does NOT include the obstacle's own size.
 * @property {number} height - Height in meters, extending up from position.z
 * @property {number} yaw - Obstacle rotation in degrees, same convention as waypoint yaw: 0=North (+Y), 90=East (+X). Only meaningful for geometry_type 'rectangle' - a circle is rotationally symmetric.
 */

/**
 * Derive the obstacle's horizontal center and base-level point (for cylinder checks and detour math).
 * @param {Obstacle} obstacle
 * @returns {Point3D}
 */
export function obstacleCenter(obstacle) {
  return { x: obstacle.position?.x ?? 0, y: obstacle.position?.y ?? 0, z: obstacle.position?.z ?? 0 };
}

/**
 * Rotate a point from world (ENU) coordinates into an obstacle's local frame,
 * undoing the obstacle's yaw so its bounds can be checked as an axis-aligned box.
 * Yaw convention matches waypoint yaw: 0deg=North(+Y), 90deg=East(+X), clockwise.
 * @param {Point3D} point - World-space point
 * @param {Point3D} center - Obstacle center (rotation origin)
 * @param {number} yawDeg - Obstacle yaw in degrees (waypoint convention)
 * @returns {Point3D} Point in the obstacle's local frame (z unchanged)
 */
export function worldToObstacleFrame(point, center, yawDeg) {
  const dx = point.x - center.x;
  const dy = point.y - center.y;
  if (!yawDeg) return { x: dx, y: dy, z: point.z };

  // Waypoint yaw is measured clockwise from +Y (North). To undo a clockwise
  // rotation by yaw, rotate the point counter-clockwise by yaw about +Y->+X.
  const rad = (yawDeg * Math.PI) / 180;
  const cos = Math.cos(rad);
  const sin = Math.sin(rad);
  return {
    x: dx * cos - dy * sin,
    y: dx * sin + dy * cos,
    z: point.z,
  };
}

/**
 * Derive a cylinder around an obstacle's real geometry, expanded by `margin`.
 * Does NOT include the obstacle's own safety_margin - callers decide whether
 * to fold it in (e.g. pass obstacle.safety_margin for the exclusion zone, or
 * obstacle.safety_margin + extra for a wider "caution" zone).
 * For geometry_type 'rectangle' this is a conservative circumscribing cylinder
 * (radius = half-diagonal), suitable for detour heuristics; use obstacleOBB
 * for the precise oriented-box collision check.
 * @param {Obstacle} obstacle
 * @param {number} [margin=0] - Extra radius/height margin in meters, around the real geometry
 * @returns {Cylinder}
 */
export function obstacleCylinder(obstacle, margin = 0) {
  const center = obstacleCenter(obstacle);
  const height = obstacle.height ?? 100;

  if (obstacle.geometry_type === 'rectangle') {
    const halfW = (obstacle.dimensions?.width ?? 0) / 2;
    const halfL = (obstacle.dimensions?.length ?? 0) / 2;
    const radius = Math.sqrt(halfW * halfW + halfL * halfL) + margin;
    return { center, radius, height: height + margin };
  }

  return { center, radius: (obstacle.dimensions?.radius ?? 0) + margin, height: height + margin };
}

/**
 * Derive the world-space AABB around an obstacle's real geometry, expanded by
 * `margin` - a tight box for 'circle' geometry, or the rotated rectangle's
 * world-space envelope for 'rectangle' geometry (used only for fast
 * rejection; the precise check is obstacleOBB). Does NOT include the
 * obstacle's own safety_margin - callers decide whether to fold it in.
 * @param {Obstacle} obstacle
 * @param {number} [margin=0] - Extra margin in meters, around the real geometry
 * @returns {AABB}
 */
export function obstacleAABB(obstacle, margin = 0) {
  const height = obstacle.height ?? 100;
  const center = obstacleCenter(obstacle);

  if (obstacle.geometry_type === 'rectangle') {
    const halfW = (obstacle.dimensions?.width ?? 0) / 2 + margin;
    const halfL = (obstacle.dimensions?.length ?? 0) / 2 + margin;
    // World-space envelope of the rotated rectangle: half-extents swap worst-case
    // under rotation, so use the diagonal as a conservative radius for fast rejection.
    const diag = Math.sqrt(halfW * halfW + halfL * halfL);
    return {
      min_point: { x: center.x - diag, y: center.y - diag, z: 0 - margin },
      max_point: { x: center.x + diag, y: center.y + diag, z: height + margin },
    };
  }

  const r = (obstacle.dimensions?.radius ?? 0) + margin;
  return {
    min_point: { x: center.x - r, y: center.y - r, z: center.z - margin },
    max_point: { x: center.x + r, y: center.y + r, z: center.z + height + margin },
  };
}

/**
 * Derive the oriented bounding box (OBB) parameters for a geometry_type
 * 'rectangle' obstacle's real geometry, expanded by `margin`: its local-frame
 * half-extents, center, yaw and height. Does NOT include the obstacle's own
 * safety_margin - callers decide whether to fold it in. Use with
 * worldToObstacleFrame() to test points/segments precisely.
 * @param {Obstacle} obstacle
 * @param {number} [margin=0] - Extra margin in meters, around the real geometry
 * @returns {{center: Point3D, halfExtent: {x:number,y:number}, height: number, yaw: number}}
 */
export function obstacleOBB(obstacle, margin = 0) {
  return {
    center: obstacleCenter(obstacle),
    halfExtent: {
      x: (obstacle.dimensions?.width ?? 0) / 2 + margin,
      y: (obstacle.dimensions?.length ?? 0) / 2 + margin,
    },
    height: (obstacle.height ?? 100) + margin,
    yaw: obstacle.yaw ?? 0,
  };
}

/**
 * Check if a point is inside an obstacle's OBB (oriented bounding box).
 * @param {Point3D} point - World-space point
 * @param {{center: Point3D, halfExtent: {x:number,y:number}, height: number, yaw: number}} obb
 * @returns {boolean}
 */
export function pointInOBB(point, obb) {
  const local = worldToObstacleFrame(point, obb.center, obb.yaw);
  return (
    Math.abs(local.x) <= obb.halfExtent.x &&
    Math.abs(local.y) <= obb.halfExtent.y &&
    point.z >= 0 &&
    point.z <= obb.height
  );
}

/**
 * Check if a line segment intersects an obstacle's OBB by rotating the
 * segment into the obstacle's local frame and running the AABB slab test.
 * @param {Segment} segment - World-space segment
 * @param {{center: Point3D, halfExtent: {x:number,y:number}, height: number, yaw: number}} obb
 * @returns {{intersects: boolean, tMin: number, tMax: number}}
 */
export function segmentIntersectsOBB(segment, obb) {
  const localSegment = {
    start: worldToObstacleFrame(segment.start, obb.center, obb.yaw),
    end: worldToObstacleFrame(segment.end, obb.center, obb.yaw),
  };
  const localAABB = {
    min_point: { x: -obb.halfExtent.x, y: -obb.halfExtent.y, z: 0 },
    max_point: { x: obb.halfExtent.x, y: obb.halfExtent.y, z: obb.height },
  };
  return segmentIntersectsAABB(localSegment, localAABB);
}

/**
 * Penetration depth of a world-space point inside an obstacle's OBB, split
 * into horizontal (xy) and vertical (z) components rather than one blended
 * distance - a route can be barely inside the footprint but deep inside the
 * height range (or vice versa), and collapsing both into a single number
 * hides which direction actually offers the shortest way out. Measured in
 * the box's local (yaw-undone) frame. Only meaningful when the point is
 * actually inside the box - callers must check pointInOBB first.
 * @param {Point3D} point - World-space point
 * @param {{center: Point3D, halfExtent: {x:number,y:number}, height: number, yaw: number}} obb
 * @returns {{xy: number, z: number}} Distance to nearest side wall (xy) and to nearest floor/ceiling (z), in meters
 */
export function penetrationDepthOBB(point, obb) {
  const local = worldToObstacleFrame(point, obb.center, obb.yaw);
  const penX = obb.halfExtent.x - Math.abs(local.x);
  const penY = obb.halfExtent.y - Math.abs(local.y);
  const distToFloor = point.z - obb.center.z;
  const distToCeiling = obb.center.z + obb.height - point.z;
  return { xy: Math.min(penX, penY), z: Math.min(distToFloor, distToCeiling) };
}

/**
 * Penetration depth of a world-space point inside a cylinder, split into
 * horizontal (xy, distance to the radial wall) and vertical (z, distance to
 * floor/ceiling) components rather than one blended distance - a route can
 * be barely inside the radius but deep inside the height range (or vice
 * versa), and collapsing both into a single number hides which direction
 * actually offers the shortest way out. The cylinder's "center" is its base,
 * not its centroid, so the z component is measured from there. Only
 * meaningful when the point is actually inside the cylinder - callers must
 * check pointInCylinder first.
 * @param {Point3D} point - World-space point
 * @param {Cylinder} cylinder
 * @returns {{xy: number, z: number}} Distance to radial wall (xy) and to nearest floor/ceiling (z), in meters
 */
export function penetrationDepthCylinder(point, cylinder) {
  const radialPen = cylinder.radius - distance2D(point, cylinder.center);
  const distToFloor = point.z - cylinder.center.z;
  const distToCeiling = cylinder.center.z + cylinder.height - point.z;
  return { xy: radialPen, z: Math.min(distToFloor, distToCeiling) };
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
