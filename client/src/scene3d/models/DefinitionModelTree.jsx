import React, { useMemo } from 'react';
import * as THREE from 'three';
import PrimitiveMesh from './definitionPrimitives.jsx';

// Revolute: axis-angle rotation, arbitrary axis — Three.js's own Quaternion
// does the composition, no matrix math of our own needed (see
// definitionResolver.js for why the server sends origin/joint separately
// instead of one pre-composed Euler triple).
const JointGroup = ({ joint, children }) => {
  const quaternion = useMemo(() => {
    if (joint.type !== 'revolute') return null;
    return new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(...joint.axis).normalize(),
      joint.angleRad,
    );
  }, [joint]);

  if (joint.type === 'revolute') return <group quaternion={quaternion}>{children}</group>;
  if (joint.type === 'prismatic') {
    const offset = joint.axis.map((v) => v * joint.distance);
    return <group position={offset}>{children}</group>;
  }
  return <>{children}</>;
};

const LinkNode = ({ link, childrenByParent }) => (
  <group position={link.origin.position} rotation={link.origin.rotationEuler}>
    <JointGroup joint={link.joint}>
      {link.geometry.map((g) => (
        <PrimitiveMesh key={g.id} primitive={g} />
      ))}
      {(childrenByParent[link.name] || []).map((child) => (
        <LinkNode key={child.name} link={child} childrenByParent={childrenByParent} />
      ))}
    </JointGroup>
  </group>
);

/**
 * Renders a resolved link tree (POST .../definition/model) as nested
 * <group>s — each link's `origin` (straight from its `xyz`/`rpy_deg`, no
 * decomposition needed) wraps a `joint` group, which wraps its geometry and
 * its own children. Three.js composes the whole parent→child chain via the
 * scene graph.
 */
const DefinitionModelTree = ({ model, position = [0, 0, 0] }) => {
  const childrenByParent = useMemo(() => {
    const map = {};
    for (const link of model.links) {
      const parent = link.parent || 'world';
      (map[parent] = map[parent] || []).push(link);
    }
    return map;
  }, [model]);

  const roots = childrenByParent.world || [];

  return (
    <group position={position}>
      {roots.map((link) => (
        <LinkNode key={link.name} link={link} childrenByParent={childrenByParent} />
      ))}
    </group>
  );
};

export default DefinitionModelTree;
