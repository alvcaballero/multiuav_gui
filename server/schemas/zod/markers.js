import { z } from 'zod';

export const ElementTypeSchema = z.object({
  id: z.string(),
  name: z.string(),
  description: z.string().optional(),
  icon: z.string().nullable().optional(),
  model3d: z.string().nullable().optional(),
  color: z.string().nullable().optional(),
  isCustom: z.boolean().optional(),
});

export const ElementGroupSchema = z.object({
  typeId: z.string(),
  name: z.string(),
  description: z.string().optional(),
  linea: z.boolean().optional(),
  attributes: z.record(z.string(), z.union([z.string(), z.number()])).optional(),
});

export const ElementItemSchema = z.object({
  groupId: z.coerce.number(),
  name: z.string(),
  latitude: z.number(),
  longitude: z.number(),
  description: z.string().nullable().optional(),
  attributes: z.record(z.string(), z.union([z.string(), z.number()])).nullable().optional(),
});

export const BaseSchema = z.object({
  id: z.string(),
  typeId: z.string().nullable().optional(),
  name: z.string().nullable().optional(),
  latitude: z.number(),
  longitude: z.number(),
  corners: z.array(z.object({ latitude: z.number(), longitude: z.number() })).nullable().optional(),
});

export const AssignmentSchema = z.object({
  baseId: z.string(),
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
