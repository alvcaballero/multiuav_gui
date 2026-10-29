import React from 'react';
import { WIRE_COLOR } from './definitionPrimitives';

export const FALLBACK_GEOMETRY = {
  geometry_type: 'circle',
  dimensions: { radius: 2, height: 5 },
};

// heading: degrees from North, clockwise (0=N, 90=E, 180=S, 270=W)
// Three.js axes: X=East, Y=up, Z=-North → rotY = -heading_rad
const headingToRotationY = (heading = 0) => -(heading * Math.PI) / 180;

// Wireframe bounding volume for an ElementType geometry. Its horizontal
// center matches the model origin, while its base sits on the ground (y=0)
// and extrudes upward by dimensions.height.
const GeometryWireframe = ({
  position = [0, 0, 0],
  geometry,
  azimFront = 0,
  color = WIRE_COLOR,
}) => {
  const { geometry_type: geometryType, dimensions } = geometry;
  const height = dimensions?.height ?? FALLBACK_GEOMETRY.dimensions.height;
  const meshPosition = [position[0], position[1] + height / 2, position[2]];
  const rotationY = headingToRotationY(azimFront);

  if (geometryType === 'rectangle') {
    const width = dimensions?.width ?? FALLBACK_GEOMETRY.dimensions.radius * 2;
    const length = dimensions?.length ?? FALLBACK_GEOMETRY.dimensions.radius * 2;
    return (
      <mesh position={meshPosition} rotation={[0, rotationY, 0]}>
        <boxGeometry args={[width, height, length]} />
        <meshBasicMaterial color={color} wireframe />
      </mesh>
    );
  }

  const radius = dimensions?.radius ?? FALLBACK_GEOMETRY.dimensions.radius;
  return (
    <mesh position={meshPosition}>
      <cylinderGeometry args={[radius, radius, height, 32]} />
      <meshBasicMaterial color={color} wireframe />
    </mesh>
  );
};

export default GeometryWireframe;
