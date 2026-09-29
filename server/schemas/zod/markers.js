import { z } from 'zod';

// Geometry lives ONLY on ElementType — dimensions are catalog data, immutable
// per item. An ElementItem/Base never carries its own geometry; a different
// size means creating/editing a different ElementType, never overriding an
// instance. (Previously `dimensions`+`yaw` were shared with ElementItem as a
// "per-item override" — that let two items of the same type diverge in size,
// which is the bug this schema now prevents structurally. `yaw` moved out
// entirely: orientation is per-instance state, see `azimFront` below.)
const CircleDimensionsSchema = z.object({
  radius: z.number().positive(),
  height: z.number().positive(),
});
const RectangleDimensionsSchema = z.object({
  width: z.number().positive(),
  length: z.number().positive(),
  height: z.number().positive(),
});
export const GeometrySchema = z.discriminatedUnion('geometry_type', [
  z.object({
    geometry_type: z.literal('circle'),
    dimensions: CircleDimensionsSchema,
  }),
  z.object({
    geometry_type: z.literal('rectangle'),
    dimensions: RectangleDimensionsSchema,
  }),
]);

// Reserved attribute keys on ElementType — a per-type parameter can't shadow
// the catalog's own `geometry`/`parameterDefs` keys.
const RESERVED_PARAMETER_KEYS = new Set(['geometry', 'parameterDefs']);

// A single configurable parameter an ElementType exposes to its items (e.g. a
// wind turbine's `nacelle_heading_deg` or `operational_status`). The TYPE
// defines the shape (this schema); each ElementItem/Base of that type stores
// its own VALUE for it under `attributes` (see ItemAttributesSchema).
export const ParameterDefSchema = z
  .object({
    key: z.string().regex(/^[a-zA-Z_][a-zA-Z0-9_]*$/, 'key must be a valid identifier'),
    label: z.string(),
    dataType: z.enum(['number', 'string', 'boolean', 'enum']),
    unit: z.string().optional(),
    options: z.array(z.string()).optional(),
    default: z.union([z.string(), z.number(), z.boolean()]).optional(),
    // Numeric range (e.g. a crane trolley's travel limits) — only meaningful
    // for dataType 'number'. Optional: most sources (a hand-authored DB
    // entry) won't have one.
    min: z.number().optional(),
    max: z.number().optional(),
    description: z.string().optional(),
  })
  .superRefine((def, ctx) => {
    if (def.dataType === 'enum' && (!def.options || def.options.length === 0)) {
      ctx.addIssue({
        code: z.ZodIssueCode.custom,
        message: 'enum parameters require at least one option',
        path: ['options'],
      });
    }
    if (RESERVED_PARAMETER_KEYS.has(def.key)) {
      ctx.addIssue({
        code: z.ZodIssueCode.custom,
        message: `"${def.key}" is a reserved key`,
        path: ['key'],
      });
    }
  });

const ParameterDefsSchema = z.array(ParameterDefSchema).superRefine((defs, ctx) => {
  const seen = new Set();
  defs.forEach((def, index) => {
    if (seen.has(def.key)) {
      ctx.addIssue({
        code: z.ZodIssueCode.custom,
        message: `duplicate parameter key "${def.key}"`,
        path: [index, 'key'],
      });
    }
    seen.add(def.key);
  });
});

// ElementType attributes: catalog data only — geometry (dimensions) and the
// per-type parameter SCHEMA (defs, not values), plus free-form scalar keys.
export const TypeAttributesSchema = z
  .object({
    geometry: GeometrySchema.optional(),
    parameterDefs: ParameterDefsSchema.optional(),
  })
  .catchall(z.union([z.string(), z.number()]));

// ElementItem/Base attributes: instance VALUES only (parameter values keyed
// by an ElementType's `parameterDefs`). No `geometry`/`parameterDefs` key is
// declared here — the catchall (scalar only) already rejects an object/array
// under any key, so a stray `attributes.geometry` fails validation with no
// extra logic needed.
export const ItemAttributesSchema = z.object({}).catchall(z.union([z.string(), z.number(), z.boolean()]));

export const ElementTypeSchema = z.object({
  // Autoincrement PK — the server assigns it on create; a client-sent id is
  // ignored there (see elementTypesModel.create). Accepted here only so a
  // GET-then-PUT roundtrip (the type editor re-saving its own fetched item)
  // doesn't fail validation.
  id: z.coerce.number().int().positive().optional(),
  name: z.string(),
  description: z.string().optional(),
  icon: z.string().nullable().optional(),
  model3d: z.string().nullable().optional(),
  // URL to a stored semantic/parametric model file (e.g. a wtsem-format
  // .type.yaml) — an asset like icon/model3d, not inline content. Its
  // `state_defaults` back `parameterDefs` when the DB copy is empty, see
  // elementTypesModel.resolveEffectiveParameterDefs.
  definitionYaml: z.string().nullable().optional(),
  color: z.string().nullable().optional(),
  isCustom: z.boolean().optional(),
  attributes: TypeAttributesSchema.nullable().optional(),
});

// Bounding box over the group's items' lat/lng — min/max corner points.
// Derived and written only by elementGroupsModel.recalculateBounds(), never
// sent by the client, but modeled here so `attributes` reflects what's
// actually persisted.
const GroupBoundsSchema = z
  .object({
    minLat: z.number(),
    maxLat: z.number(),
    minLng: z.number(),
    maxLng: z.number(),
  })
  .nullable();

export const ElementGroupSchema = z.object({
  typeId: z.coerce.number().int().positive(),
  name: z.string(),
  description: z.string().optional(),
  linea: z.boolean().optional(),
  attributes: z
    .object({ bounds: GroupBoundsSchema.optional() })
    .catchall(z.union([z.string(), z.number()]))
    .optional(),
});

export const ElementItemSchema = z.object({
  groupId: z.coerce.number(),
  name: z.string(),
  latitude: z.number(),
  longitude: z.number(),
  // Not MSL — same relative-to-origin frame as everywhere else in this system
  // (see coordinateConverter.js: origin.alt is the average GPS altitude the
  // connected drones reported, there's no independent MSL reference).
  altitude: z.number().optional(),
  // Orientation: the single source of truth for "which way this instance
  // faces" — replaces the old `attributes.geometry.yaw` (removed, was
  // catalog-mixed-with-state) and the legacy client-only `heading` field
  // (never persisted). 0=North, 90=East.
  azimFront: z.number().min(0).max(360).optional(),
  description: z.string().nullable().optional(),
  attributes: ItemAttributesSchema.nullable().optional(),
});

export const BaseSchema = z.object({
  id: z.coerce.number().int().positive().optional(),
  typeId: z.coerce.number().int().positive().nullable().optional(),
  name: z.string().nullable().optional(),
  latitude: z.number(),
  longitude: z.number(),
  altitude: z.number().optional(),
  azimFront: z.number().min(0).max(360).optional(),
  attributes: ItemAttributesSchema.nullable().optional(),
  corners: z
    .array(z.object({ latitude: z.number(), longitude: z.number() }))
    .nullable()
    .optional(),
});

export const AssignmentSchema = z.object({
  baseId: z.coerce.number().int().positive(),
  deviceId: z.coerce.number(),
  settings: z.record(z.string(), z.union([z.string(), z.number(), z.array(z.any())])).optional(),
});

export function validateElementType(input) {
  return ElementTypeSchema.safeParse(input);
}
export function validatePartialElementType(input) {
  return ElementTypeSchema.partial().safeParse(input);
}

export function validateElementGroup(input) {
  return ElementGroupSchema.safeParse(input);
}
export function validatePartialElementGroup(input) {
  return ElementGroupSchema.partial().safeParse(input);
}

export function validateElementItem(input) {
  return ElementItemSchema.safeParse(input);
}
export function validatePartialElementItem(input) {
  return ElementItemSchema.partial().safeParse(input);
}

export function validateBase(input) {
  return BaseSchema.safeParse(input);
}
export function validatePartialBase(input) {
  return BaseSchema.partial().safeParse(input);
}

export function validateAssignment(input) {
  return AssignmentSchema.safeParse(input);
}
export function validatePartialAssignment(input) {
  return AssignmentSchema.partial().safeParse(input);
}
