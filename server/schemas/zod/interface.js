import { z } from 'zod';
import { FIRMWARE, INTERFACE_CONTROL, VEHICLE_TYPE } from '../../config/status.js';

// Shape of ONE interface file (config/devices/interfaces/<key>.yaml), as written: strict, so a
// typo (`nmae:`, `messageTyp:`) or a param nobody reads is an error instead of a silent no-op.
// What only makes sense once the file is resolved (a messageType on every entry, `name` per
// message format) is checked by interfaceCatalog.js, not here.

const legacyType = z.never({ errorMap: () => ({ message: "renamed: use 'messageType'" }) }).optional();

const rosName = z
  .string()
  .regex(/^\/\S*$/, "must start with '/' and contain no spaces");

const endpoint = {
  name: rosName.optional(),
  messageType: z.string().min(1).optional(),
  serviceType: legacyType,
  actionType: legacyType,
};

// Params a decoder reads from its subscriber entry (msgDecode.js) and the watchdog's own.
const subscriber = z
  .object({
    ...endpoint,
    watchdog: z.boolean().optional(),
    // Not subscribed with the device: a viewer opens it (models/lidar/lidarStream.js)
    on_demand: z.boolean().optional(),
    timeout_s: z.number().positive().optional(),
    rate_hz: z.number().positive().optional(),
    scale: z.number().optional(),
    unit: z.enum(['rad', 'deg']).optional(),
    heading_offset: z.number().optional(),
    priority: z.number().optional(),
    fresh_ms: z.number().int().positive().optional(),
    attribute: z.string().regex(/^[A-Za-z0-9_]+$/).optional(),
  })
  .strict();

const command = z.object({ ...endpoint }).strict();

const heartbeat = z
  .object({
    name: rosName.optional(),
    messageType: z.string().min(1).optional(),
    rate_hz: z.number().positive().optional(),
    message: z.record(z.unknown()).optional(),
  })
  .strict();

// `null` removes an entry the interface inherits (`extends`).
const block = (entry) => z.record(z.string(), entry.nullable());

const enumOf = (values) => z.enum(Object.values(values));
const uniqueList = (item, message) =>
  z
    .array(item)
    .min(1)
    .refine((list) => new Set(list).size === list.length, message);

export const interfaceSchema = z
  .object({
    // Inherits every block of that interface; an entry here is merged over the parent's.
    extends: z.string().regex(/^[A-Za-z0-9_]+$/).optional(),
    // Only a base to extend: no device may use it, and it is not listed.
    abstract: z.boolean().optional(),
    firmwares: uniqueList(enumOf(FIRMWARE), 'repeated firmware').optional(),
    vehicleTypes: uniqueList(enumOf(VEHICLE_TYPE), 'repeated vehicle type').optional(),
    control: enumOf(INTERFACE_CONTROL).optional(),
    // The device model a device of this interface gets when none is chosen (config/devices/
    // device_models/). Without it: the generic model of its first vehicle type.
    defaultModel: z.string().regex(/^[A-Za-z0-9_]+$/).optional(),
    subscribers: block(subscriber).optional(),
    publishers: block(command).optional(),
    services: block(command).optional(),
    actions: block(command).optional(),
    heartbeat: heartbeat.nullable().optional(),
    // Narrows command params (commandSchema.validateCommandOverrides checks the inside).
    commands: z.record(z.string(), z.record(z.unknown()).nullable()).optional(),
    // mission_schema.yaml profile, when the one derived from configure_mission is not right.
    mission: z.string().min(1).optional(),
  })
  .strict();

/** @returns {{path: (string|number)[], message: string}[]} empty when `def` is well formed */
export function interfaceShapeProblems(def) {
  const result = interfaceSchema.safeParse(def ?? {});
  if (result.success) return [];
  return result.error.issues.map(({ path, message }) => ({ path, message }));
}
