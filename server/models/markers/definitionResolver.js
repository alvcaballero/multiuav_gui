// Resolves a type's stored .yaml semantic/parametric model (wtsem-type/0.2's
// `state_defaults`, or insem/0.2's `state`/`links`) into a plain JSON tree
// the client can map directly to React Three Fiber `<group>`/`<mesh>`
// elements — no expressions, no YAML, left for the client to parse.
//
// Deliberately does NOT compose origin-rotation and joint-rotation into a
// single matrix and decompose it back to one Euler triple server-side (that
// needs general matrix→Euler decomposition, which has real edge cases at
// gimbal lock). Instead each link carries its `origin` (position +
// rotationEuler, straight from `xyz`/`rpy_deg` — no decomposition needed,
// it's already an Euler triple) and its `joint` (axis + angle for revolute,
// axis + distance for prismatic) SEPARATELY. The client nests two groups per
// link and lets Three.js's own Quaternion/axis-angle machinery do the
// arbitrary-axis composition — exactly what it's already good at.
//
// Verified against /home/grvc/work/px4/wtsem/insem/insem.py (reference only,
// not modified): `ev()` (namespace/expression eval), `_kinematics()` (joint
// composition — this resolver computes only the LOCAL per-link transform,
// skipping the `done[parent] @` world-accumulation step, since a link's own
// origin/joint never depends on another link's resolved transform — only on
// the shared `parameters`/`state`/`derived` namespace), `build_prim`
// (primitive shapes), `Table` (swept_box chord/thickness interpolation).

import { parse } from 'yaml';
import fs from 'fs';
import { elementTypesModel } from './elementTypes.js';
import { evaluateExpression } from './safeExpr.js';

export class DefinitionResolveError extends Error {
  constructor(message, { link = null, expression = null } = {}) {
    super(message);
    this.name = 'DefinitionResolveError';
    this.link = link;
    this.expression = expression;
  }
}

function evalScalar(x, ns) {
  if (x === undefined || x === null) return undefined;
  if (typeof x === 'number') return x;
  if (typeof x === 'boolean') return x ? 1 : 0;
  if (typeof x === 'string') {
    try {
      return evaluateExpression(x, ns);
    } catch (err) {
      throw new DefinitionResolveError(err.message, { expression: x });
    }
  }
  throw new DefinitionResolveError(`Expected a number or expression, got ${JSON.stringify(x)}`, {
    expression: String(x),
  });
}

function evalArray(arr, ns, fallback) {
  const source = arr ?? fallback;
  if (!Array.isArray(source)) {
    throw new DefinitionResolveError(`Expected an array, got ${JSON.stringify(source)}`);
  }
  return source.map((v) => evalScalar(v, ns));
}

// `origin`/`pose` blocks share the same shape: `{xyz: [...], rpy_deg: [...]}`.
// rpy_deg (roll, pitch, yaw, degrees) maps directly to Three.js's default
// Euler order 'XYZ' in radians — verified against insem.py's `rpy_matrix`
// (`Rz(yaw) @ Ry(pitch) @ Rx(roll)`), the same composition Three.js's
// default XYZ Euler order builds.
function evalPose(pose, ns) {
  const position = evalArray(pose?.xyz, ns, [0, 0, 0]);
  const rpyDeg = evalArray(pose?.rpy_deg, ns, [0, 0, 0]);
  const rotationEuler = rpyDeg.map((deg) => (deg * Math.PI) / 180);
  return { position, rotationEuler };
}

function vecSub(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function vecLength(v) {
  return Math.hypot(v[0], v[1], v[2]);
}

function unitVec(v) {
  const len = vecLength(v) || 1;
  return [v[0] / len, v[1] / len, v[2] / len];
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

// A `capsule`/`beam` primitive runs from point `a` to point `b` (its own
// local Z axis), not a fixed `pose` — this builds the rotation that aligns
// canonical +Z with that direction, with `up` fixing the twist around it
// (defaults matching insem.py's `frame_from_axis`: world +Z, or +X if the
// segment is nearly vertical). Verified 1:1 against insem.py lines 112-119.
function frameFromAxis(a, b, up) {
  const z = unitVec(vecSub(b, a));
  const upVec = up ?? (Math.abs(z[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0]);
  const x = unitVec(
    vecSub(
      upVec,
      z.map((v) => v * dot(upVec, z))
    )
  );
  const y = cross(z, x);
  // Rotation matrix with columns [x, y, z] — m[row][col].
  return [
    [x[0], y[0], z[0]],
    [x[1], y[1], z[1]],
    [x[2], y[2], z[2]],
  ];
}

// Standard (Shepperd) rotation-matrix -> quaternion conversion — numerically
// stable across all rotations, unlike a matrix -> Euler decomposition (no
// gimbal-lock edge case to worry about). Returns [x, y, z, w].
function matrixToQuaternion(m) {
  const [[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]] = m;
  const trace = m00 + m11 + m22;
  if (trace > 0) {
    const s = 0.5 / Math.sqrt(trace + 1);
    return [(m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s, 0.25 / s];
  }
  if (m00 > m11 && m00 > m22) {
    const s = 2 * Math.sqrt(1 + m00 - m11 - m22);
    return [0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s];
  }
  if (m11 > m22) {
    const s = 2 * Math.sqrt(1 + m11 - m00 - m22);
    return [(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s];
  }
  const s = 2 * Math.sqrt(1 + m22 - m00 - m11);
  return [(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s];
}

function linterp(f, xs, ys) {
  if (f <= xs[0]) return ys[0];
  if (f >= xs[xs.length - 1]) return ys[ys.length - 1];
  for (let i = 0; i < xs.length - 1; i++) {
    if (f >= xs[i] && f <= xs[i + 1]) {
      const t = (f - xs[i]) / (xs[i + 1] - xs[i]);
      return ys[i] + t * (ys[i + 1] - ys[i]);
    }
  }
  return ys[ys.length - 1];
}

// A `width`/`depth` spec on a swept_box is one of: `{table: "name"}`
// (reference into the top-level `tables:` block), `{stations, values}`
// (inline table), or a plain scalar/expression (constant across the span).
function buildTable(spec, ns, resolvedTables) {
  if (spec && typeof spec === 'object' && !Array.isArray(spec) && 'table' in spec) {
    const named = resolvedTables[spec.table];
    if (!named) throw new DefinitionResolveError(`Unknown table "${spec.table}"`);
    return (f) => linterp(f, named.stations, named.values);
  }
  if (spec && typeof spec === 'object' && !Array.isArray(spec) && ('stations' in spec || 'values' in spec)) {
    const stations = evalArray(spec.stations, ns);
    const values = evalArray(spec.values, ns);
    return (f) => linterp(f, stations, values);
  }
  const constant = evalScalar(spec, ns);
  return () => constant;
}

function resolveTables(doc, ns) {
  const out = {};
  for (const [key, spec] of Object.entries(doc.tables || {})) {
    out[key] = { stations: evalArray(spec.stations, ns), values: evalArray(spec.values, ns) };
  }
  return out;
}

// `state_defaults` (wtsem-type/0.2, flat {key: scalar}) or `state`
// (insem/0.2, {key: scalar} OR {key: {value, unit?, limits?, ...}} — mixed
// within the same file). Only NUMERIC entries join the arithmetic namespace
// — a string like `operational_status: parked` can't be used in an
// expression, same as insem.py's `ns` construction (line 341).
function buildNamespace(doc) {
  const ns = {};
  for (const [key, value] of Object.entries(doc.parameters || {})) {
    ns[key] = evalScalar(value, ns);
  }
  const state = doc.state || doc.state_defaults || {};
  for (const [key, entry] of Object.entries(state)) {
    const value = entry && typeof entry === 'object' && !Array.isArray(entry) && 'value' in entry ? entry.value : entry;
    if (typeof value === 'number') ns[key] = value;
  }
  for (const [key, expr] of Object.entries(doc.derived || {})) {
    try {
      ns[key] = evalScalar(expr, ns);
    } catch (err) {
      if (err instanceof DefinitionResolveError) {
        err.link = err.link ?? `derived.${key}`;
      }
      throw err;
    }
  }
  return ns;
}

// `capsule`/`beam` don't have a `pose` at all — they run from point `a` to
// point `b`, so their position/orientation comes from `frameFromAxis`
// instead of `evalPose`. Mirrors insem.py's `build_prim` (`T = H(frame_from_axis(a, b, g.get("up")), a)`).
function resolveAxisPrimitive(g, ns) {
  const a = evalArray(g.a, ns);
  const b = evalArray(g.b, ns);
  const up = g.up ? evalArray(g.up, ns) : undefined;
  const quaternion = matrixToQuaternion(frameFromAxis(a, b, up));
  const length = vecLength(vecSub(b, a));
  return { position: a, quaternion, length };
}

function resolveGeometryItem(g, index, ns, resolvedTables) {
  const id = g.id ?? `p${index}`;
  switch (g.type) {
    case 'box': {
      const { position, rotationEuler } = evalPose(g.pose, ns);
      return { id, type: 'box', size: evalArray(g.size, ns), position, rotationEuler };
    }
    case 'sphere': {
      const { position, rotationEuler } = evalPose(g.pose, ns);
      return { id, type: 'sphere', radius: evalScalar(g.radius, ns), position, rotationEuler };
    }
    case 'cylinder': {
      const { position, rotationEuler } = evalPose(g.pose, ns);
      const radiusBottom = evalScalar(g.radius_bottom ?? g.radius, ns);
      const radiusTop = evalScalar(g.radius_top ?? g.radius, ns);
      return {
        id,
        type: 'cylinder',
        radiusBottom,
        radiusTop,
        length: evalScalar(g.length, ns),
        position,
        rotationEuler,
      };
    }
    case 'swept_box': {
      const { position, rotationEuler } = evalPose(g.pose, ns);
      const length = evalScalar(g.length, ns);
      const segmentCount = Math.max(1, Math.round(evalScalar(g.segments ?? 20, ns)));
      const widthTable = buildTable(g.width, ns, resolvedTables);
      const depthTable = buildTable(g.depth, ns, resolvedTables);
      const segments = [];
      for (let i = 0; i < segmentCount; i++) {
        const z0 = (length * i) / segmentCount;
        const z1 = (length * (i + 1)) / segmentCount;
        const w = Math.max(widthTable(z0 / length), widthTable(z1 / length));
        const d = Math.max(depthTable(z0 / length), depthTable(z1 / length));
        segments.push({ position: [0, 0, (z0 + z1) / 2], size: [d, w, z1 - z0] });
      }
      return { id, type: 'swept_box', position, rotationEuler, segments };
    }
    case 'capsule': {
      const { position, quaternion, length } = resolveAxisPrimitive(g, ns);
      return { id, type: 'capsule', radius: evalScalar(g.radius, ns), length, position, quaternion };
    }
    case 'beam': {
      const { position, quaternion, length } = resolveAxisPrimitive(g, ns);
      const width = evalScalar(g.width, ns);
      const depth = evalScalar(g.depth ?? g.width, ns);
      return { id, type: 'beam', size: [depth, width], length, position, quaternion };
    }
    default:
      throw new DefinitionResolveError(`Unsupported primitive type "${g.type}"`);
  }
}

function resolveJoint(joint, ns) {
  const type = joint?.type ?? 'fixed';
  if (type === 'fixed') return { type: 'fixed' };
  if (type === 'revolute') {
    const axis = evalArray(joint.axis, ns, [0, 0, 1]);
    const angleDeg = evalScalar(joint.value, ns) ?? 0;
    return { type: 'revolute', axis, angleRad: (angleDeg * Math.PI) / 180 };
  }
  if (type === 'prismatic') {
    const axis = evalArray(joint.axis, ns, [1, 0, 0]);
    const norm = Math.hypot(...axis) || 1;
    const distance = evalScalar(joint.value, ns) ?? 0;
    return { type: 'prismatic', axis: axis.map((v) => v / norm), distance };
  }
  throw new DefinitionResolveError(`Unsupported joint type "${type}"`);
}

function resolveLink(link, ns, resolvedTables) {
  try {
    const origin = evalPose(link.joint?.origin, ns);
    const joint = resolveJoint(link.joint, ns);
    const rawGeometry = Array.isArray(link.geometry) ? link.geometry : link.geometry ? [link.geometry] : [];
    const geometry = rawGeometry.map((g, i) => resolveGeometryItem(g, i, ns, resolvedTables));
    return { name: link.name, parent: link.parent || 'world', origin, joint, geometry };
  } catch (err) {
    if (err instanceof DefinitionResolveError) {
      err.link = err.link ?? link.name;
      throw err;
    }
    throw new DefinitionResolveError(err.message, { link: link.name });
  }
}

/**
 * Resolves a type's definition file into a plain link tree, evaluating every
 * expression against `parameters`+`state`+`derived`. `content`, when given,
 * is evaluated directly (e.g. an unsaved edit in the client's YAML editor) —
 * otherwise the type's stored file on disk is read. Throws
 * DefinitionResolveError with `{link, expression, message}` on the first
 * evaluation failure — never a partial/best-effort result.
 */
export function resolveDefinitionModel(id, content) {
  let source = content;
  if (source === undefined) {
    const filePath = elementTypesModel.getAssetPath(id, 'definition');
    if (!filePath) return null;
    source = fs.readFileSync(filePath, 'utf8');
  }

  let doc;
  try {
    // `merge: true` — the wind turbine's blade_A/B/C links share their
    // geometry/semantic/inspection via a YAML merge key (`<<: *blade_template`,
    // see windTurbine's definition.yaml). Without this option the `yaml`
    // package leaves `<<` as a literal key instead of expanding it, so
    // `link.geometry` silently comes back undefined for every merged link.
    doc = parse(source, { merge: true });
  } catch (err) {
    throw new DefinitionResolveError(`Invalid YAML: ${err.message}`);
  }

  const ns = buildNamespace(doc);
  const resolvedTables = resolveTables(doc, ns);
  const links = (doc.links || []).map((link) => resolveLink(link, ns, resolvedTables));

  return { format: doc.format ?? null, links };
}
