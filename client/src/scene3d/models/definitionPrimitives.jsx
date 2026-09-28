import React from 'react';

const SEGMENTS = 24;
const WIRE_COLOR = '#ff9800'; // same convention as GeometryWireframe (R3DMarkers.jsx) —
// wireframe + orange visually marks this as the YAML-derived approximation,
// distinct from the solid/textured GLB it's being compared against.

const Box = ({ size }) => (
  <mesh>
    <boxGeometry args={size} />
    <meshBasicMaterial color={WIRE_COLOR} wireframe />
  </mesh>
);

const Sphere = ({ radius }) => (
  <mesh>
    <sphereGeometry args={[radius, SEGMENTS, SEGMENTS / 2]} />
    <meshBasicMaterial color={WIRE_COLOR} wireframe />
  </mesh>
);

// insem's cylinder convention: base at the primitive's own origin, extends
// along local +Z by `length`. Three.js's CylinderGeometry is Y-up and
// centered — rotate 90° around X (Y axis -> Z axis) and shift up by
// length/2 so the base lands at z=0 instead of the geometry's center.
const Cylinder = ({ radiusBottom, radiusTop, length }) => (
  <group rotation={[Math.PI / 2, 0, 0]} position={[0, 0, length / 2]}>
    <mesh>
      <cylinderGeometry args={[radiusTop, radiusBottom, length, SEGMENTS]} />
      <meshBasicMaterial color={WIRE_COLOR} wireframe />
    </mesh>
  </group>
);

const SweptBox = ({ segments }) => (
  <>
    {segments.map((seg, i) => (
      <mesh key={i} position={seg.position}>
        <boxGeometry args={seg.size} />
        <meshBasicMaterial color={WIRE_COLOR} wireframe />
      </mesh>
    ))}
  </>
);

// `capsule`/`beam` run from the primitive's own origin along local +Z by
// `length`, same base-at-origin convention as Cylinder above — same Y-up ->
// Z-up-from-base fix applies.
const Capsule = ({ radius, length }) => (
  <group rotation={[Math.PI / 2, 0, 0]} position={[0, 0, length / 2]}>
    <mesh>
      <capsuleGeometry args={[radius, Math.max(length - 2 * radius, 0), 4, SEGMENTS]} />
      <meshBasicMaterial color={WIRE_COLOR} wireframe />
    </mesh>
  </group>
);

const Beam = ({ size, length }) => (
  <mesh position={[0, 0, length / 2]}>
    <boxGeometry args={[size[0], size[1], length]} />
    <meshBasicMaterial color={WIRE_COLOR} wireframe />
  </mesh>
);

/**
 * Renders one resolved primitive (see server/models/markers/
 * definitionResolver.js) at its own pose within its link. `box`/`sphere`/
 * `cylinder`/`swept_box` carry `rotationEuler` (from a `pose`); `capsule`/
 * `beam` carry `quaternion` instead (from `frameFromAxis`, a->b) — a
 * <group> takes either.
 */
const PrimitiveMesh = ({ primitive }) => (
  <group
    position={primitive.position}
    rotation={primitive.rotationEuler}
    quaternion={primitive.quaternion}
  >
    {primitive.type === 'box' && <Box size={primitive.size} />}
    {primitive.type === 'sphere' && <Sphere radius={primitive.radius} />}
    {primitive.type === 'cylinder' && (
      <Cylinder
        radiusBottom={primitive.radiusBottom}
        radiusTop={primitive.radiusTop}
        length={primitive.length}
      />
    )}
    {primitive.type === 'swept_box' && <SweptBox segments={primitive.segments} />}
    {primitive.type === 'capsule' && (
      <Capsule radius={primitive.radius} length={primitive.length} />
    )}
    {primitive.type === 'beam' && <Beam size={primitive.size} length={primitive.length} />}
  </group>
);

export default PrimitiveMesh;
