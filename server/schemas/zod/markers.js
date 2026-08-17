import { z } from 'zod';

// Geometry defaults shared by ElementType (catalog defaults) and ElementItem
// (per-item override, copied from its type on creation). Aligned with the
// `Obstacle` typedef in models/collision/geometry.js (geometry_type/dimensions/
// yaw) so this can feed the collision engine later without a translation layer.
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
    yaw: z.number().optional(),
  }),
  z.object({
    geometry_type: z.literal('rectangle'),
    dimensions: RectangleDimensionsSchema,
    yaw: z.number().optional(),
  }),
]);

// `attributes` is a free-form bag (string|number) that can additionally carry a
// typed `geometry` key.
export const AttributesSchema = z
  .object({ geometry: GeometrySchema.optional() })
  .catchall(z.union([z.string(), z.number()]));

export const ElementTypeSchema = z.object({
  id: z.string(),
  name: z.string(),
  description: z.string().optional(),
  icon: z.string().nullable().optional(),
  model3d: z.string().nullable().optional(),
  color: z.string().nullable().optional(),
  isCustom: z.boolean().optional(),
  attributes: AttributesSchema.nullable().optional(),
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
  typeId: z.string(),
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
  description: z.string().nullable().optional(),
  attributes: AttributesSchema.nullable().optional(),
});

export const BaseSchema = z.object({
  id: z.coerce.number().int().positive().optional(),
  typeId: z.string().nullable().optional(),
  name: z.string().nullable().optional(),
  latitude: z.number(),
  longitude: z.number(),
  corners: z.array(z.object({ latitude: z.number(), longitude: z.number() })).nullable().optional(),
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
