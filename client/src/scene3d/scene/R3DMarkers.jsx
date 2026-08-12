import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { useThree } from '@react-three/fiber';
import { useModelLoader } from '../models/ModelLoader.jsx';
import { useSelector } from 'react-redux';
import { LatLon2XYZ } from '../core/convertion';
import { useMarkerTypes } from '../../hooks/useMarkerTypes';

// heading: degrees from North, clockwise (0=N, 90=E, 180=S, 270=W)
// Three.js axes: X=East, Y=up, Z=-North → rotY = -heading_rad
const headingToRotationY = (heading = 0) => -(heading * Math.PI) / 180;

// Used when neither the item nor its ElementType carry a geometry yet (not
// migrated / never set) so the wireframe still renders something sane.
const FALLBACK_GEOMETRY = { geometry_type: 'circle', dimensions: { radius: 2, height: 5 }, yaw: 0 };

// Wireframe bounding box for an item's effective geometry (own attributes.geometry,
// falling back to its ElementType's default). Horizontal center matches the
// model's XZ position; the base sits on the ground (y=0) and extrudes upward
// by dimensions.height — combining the "center = geometry center" requirement
// (XY) with the real-world fact that objects sit on the ground, not float
// centered on it (Z).
const GeometryWireframe = ({ position, geometry }) => {
  const { geometry_type: geometryType, dimensions, yaw = 0 } = geometry;
  const height = dimensions?.height ?? FALLBACK_GEOMETRY.dimensions.height;
  const meshPosition = [position[0], position[1] + height / 2, position[2]];
  const rotationY = headingToRotationY(yaw);

  if (geometryType === 'rectangle') {
    const width = dimensions?.width ?? FALLBACK_GEOMETRY.dimensions.radius * 2;
    const length = dimensions?.length ?? FALLBACK_GEOMETRY.dimensions.radius * 2;
    return (
      <mesh position={meshPosition} rotation={[0, rotationY, 0]}>
        <boxGeometry args={[width, height, length]} />
        <meshBasicMaterial color="#ff9800" wireframe />
      </mesh>
    );
  }

  const radius = dimensions?.radius ?? FALLBACK_GEOMETRY.dimensions.radius;
  return (
    <mesh position={meshPosition}>
      <cylinderGeometry args={[radius, radius, height, 32]} />
      <meshBasicMaterial color="#ff9800" wireframe />
    </mesh>
  );
};

const Marker = ({ item, onPick, showBoundingBox }) => {
  // react-doctor/no-event-handler false positive: useModelLoader's internal effect
  // does an async GLTF fetch with a module-level cache, not something a click/submit
  // handler could trigger directly — see ModelLoader.jsx's useModelLoader.
  const { model } = useModelLoader(item.type);
  const { invalidate } = useThree();
  const { types: markerTypes } = useMarkerTypes();

  // Clone is created inside useMemo so React owns the lifecycle — safe with Strict Mode.
  // Position/rotation are set here too (not in a separate effect): `clone` is a plain
  // JS object available immediately during render, not a ref that only exists post-commit,
  // so there's no need to wait an extra render to place it correctly.
  const clone = useMemo(() => {
    if (!model) return null;
    const c = model.scene.clone();
    c.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = false;
        child.receiveShadow = false;
        // Let the PBR material pick up the scene environment map.
        if (child.material) {
          const mats = Array.isArray(child.material) ? child.material : [child.material];
          mats.forEach((m) => {
            if (m.isMeshStandardMaterial) {
              m.envMapIntensity = 1.2;
              m.needsUpdate = true;
            }
          });
        }
      }
    });
    return c;
  }, [model]);

  if (clone) {
    clone.position.set(...item.pos);
    clone.rotation.set(0, headingToRotationY(item.heading), 0);
  }

  // Dispose the clone when it's replaced or the marker unmounts.
  useEffect(() => {
    return () => {
      if (clone) {
        clone.traverse((child) => {
          if (child.isMesh) {
            child.geometry.dispose();
            if (Array.isArray(child.material)) child.material.forEach((m) => m.dispose());
            else child.material.dispose();
          }
        });
      }
    };
  }, [clone]);

  // Tell R3F to draw a frame whenever the model, position, or heading change.
  useEffect(() => {
    if (clone) invalidate();
  }, [clone, item.pos, item.heading, invalidate]);

  if (!clone) return null;

  // R3F hace el raycasting: `event.point` es la coordenada 3D del mundo donde el
  // rayo del mouse tocó la superficie del modelo. `stopPropagation` evita que un
  // solo click atraviese varios modelos y dispare varios picks.
  const handleClick = onPick
    ? (event) => {
        event.stopPropagation();
        // Le paso el punto de impacto + el item de dominio al que pertenece.
        onPick(event.point, item, event);
      }
    : undefined;

  // Bases aren't ElementItems (item.type is the literal string 'base', not a
  // real catalog id), so there's no ElementType to resolve geometry from.
  const typeGeometry = markerTypes.find((t) => t.id === item.type)?.attributes?.geometry;
  const geometry =
    item.type !== 'base' ? item.attributes?.geometry || typeGeometry || FALLBACK_GEOMETRY : null;

  return (
    <>
      <primitive
        object={clone}
        onClick={handleClick}
        onPointerOver={onPick ? () => (document.body.style.cursor = 'pointer') : undefined}
        onPointerOut={onPick ? () => (document.body.style.cursor = 'auto') : undefined}
      />
      {showBoundingBox && geometry && <GeometryWireframe position={item.pos} geometry={geometry} />}
    </>
  );
};

function list2Points(mylist) {
  const waypoints = [];
  if (mylist?.elements) {
    mylist.elements.forEach((conjunto, index_cj) => {
      conjunto.items.forEach((items, item_index) => {
        waypoints.push({ ...items, type: conjunto.type, title: `${index_cj}-${item_index}` });
      });
    });
  }
  if (mylist?.bases) {
    mylist.bases.forEach((items, item_index) => {
      waypoints.push({ ...items, type: 'base', title: item_index });
    });
  }
  return waypoints;
}

// Esfera roja que nace en el punto clickeado. Primitiva propia, sin dependencias.
const PickedSphere = ({ position }) => (
  <mesh position={position}>
    <sphereGeometry args={[0.3, 16, 16]} />
    <meshStandardMaterial color="#ff2d2d" emissive="#ff2d2d" emissiveIntensity={0.4} />
  </mesh>
);

const R3DMarkers = ({ elements, showBoundingBoxes = true }) => {
  const origin3d = useSelector((state) => state.session.scene3d.origin);
  const range = useSelector((state) => state.session.scene3d.range);
  const { invalidate } = useThree();

  // Puntos creados por click sobre los markers (excluye los de type 'base').
  const [pickedPoints, setPickedPoints] = useState([]);

  const handlePick = useCallback(
    (point, item, event) => {
      // --- Info del PUNTO (raycaster de Three.js) ---
      // point:        Vector3 exacto del impacto en el mundo (ENU local)
      // event.face:   triángulo tocado + su normal (orientación de la superficie)
      // event.distance: distancia cámara → punto
      // event.uv:     coordenada de textura en el impacto (si el mesh tiene UVs)
      // event.object: el mesh hoja del GLTF que se tocó
      // event.eventObject: el <primitive> completo (el modelo entero)
      console.log('[3D pick] punto de impacto (mundo ENU):', {
        x: point.x,
        y: point.y,
        z: point.z,
      });
      console.log('[3D pick] item de dominio al que pertenece:', item);
      console.log('[3D pick] datos del raycaster:', {
        distancia: event.distance,
        normalCara: event.face?.normal,
        uv: event.uv,
        meshTocado: event.object?.name || '(sin nombre)',
        modeloTipo: item.type,
        modeloTitulo: item.title,
      });

      setPickedPoints((prev) => [
        ...prev,
        {
          // Punto 3D del mundo
          pos: [point.x, point.y, point.z],
          // El item de dominio al que pertenece (type, title, lat/lon, heading…)
          owner: item,
          // Distancia del click (útil para depurar / ordenar por cercanía)
          distance: event.distance,
        },
      ]);
      invalidate();
    },
    [invalidate],
  );

  const markers = useMemo(() => {
    const listelemnts = list2Points(elements);
    const pos = listelemnts.map(({ latitude, longitude }) => ({
      lng: longitude,
      lat: latitude,
      alt: 0,
    }));
    const posxyz = LatLon2XYZ(origin3d, pos);
    const elementxyz = listelemnts.map((element, index) => ({
      ...element,
      pos: [posxyz[index][0], posxyz[index][2], -posxyz[index][1]],
    }));
    return elementxyz.filter(
      (item) =>
        item.pos[0] > -range && item.pos[0] < range && item.pos[2] > -range && item.pos[2] < range,
    );
  }, [origin3d, elements, range]);

  useEffect(() => {
    invalidate();
  }, [markers, invalidate]);

  return (
    <>
      {markers.map((item) => (
        <Marker
          key={`${item.type}-${item.title}`}
          item={item}
          onPick={item.type === 'base' ? undefined : handlePick}
          showBoundingBox={showBoundingBoxes}
        />
      ))}
      {pickedPoints.map((pick, index) => (
        <PickedSphere key={`picked-${index}`} position={pick.pos} />
      ))}
    </>
  );
};

export default R3DMarkers;
