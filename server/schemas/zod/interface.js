import { z } from 'zod';

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

const block = (entry) => z.record(z.string(), entry);

export const interfaceSchema = z
  .object({
    subscribers: block(subscriber).optional(),
    publishers: block(command).optional(),
    services: block(command).optional(),
    actions: block(command).optional(),
    heartbeat: heartbeat.optional(),
    // Narrows command params (commandSchema.validateCommandOverrides checks the inside).
    commands: z.record(z.string(), z.record(z.unknown())).optional(),
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
