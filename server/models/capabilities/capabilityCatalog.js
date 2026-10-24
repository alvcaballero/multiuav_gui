// The capability catalog (config/devices/capabilities.yaml): what a vehicle can do, and which commands
// and telemetry slots implement it. Pure: the device models say which capabilities a vehicle has,
// the interfaces which ones they implement; this module only knows the vocabulary and the rules.
import { z } from 'zod';
import { CommandType } from '../../config/commandTypes.js';
import { KNOWN_SUBSCRIBER_KEYS } from '../../config/deviceMsgCatalog.js';
import { commandRegistry } from '../commands/commandRegistry.js';

const capabilitySchema = z
  .object({
    label: z.string().min(1),
    description: z.string().optional(),
    commands: z.array(z.enum(Object.values(CommandType))).optional(),
    slots: z.array(z.string()).optional(),
  })
  .strict();

/**
 * Validates the parsed catalog: known commands and slots, and each one in at most one capability
 * (a command in two would be gated by whichever is checked first).
 * @returns {{ catalog: Record<string, object>, problems: {path: string[], message: string}[] }}
 */
export function buildCapabilityCatalog(raw) {
  const problems = [];
  const catalog = {};
  const owner = new Map();
  for (const [key, def] of Object.entries(raw ?? {})) {
    const parsed = capabilitySchema.safeParse(def);
    if (!parsed.success) {
      for (const { path, message } of parsed.error.issues) problems.push({ path: [key, ...path], message });
      continue;
    }
    for (const slot of parsed.data.slots ?? []) {
      if (!KNOWN_SUBSCRIBER_KEYS.has(slot)) problems.push({ path: [key, 'slots'], message: `unknown slot '${slot}'` });
    }
    for (const item of [...(parsed.data.commands ?? []), ...(parsed.data.slots ?? []).map((s) => `slot:${s}`)]) {
      if (owner.has(item)) problems.push({ path: [key], message: `'${item}' already belongs to '${owner.get(item)}'` });
      else owner.set(item, key);
    }
    catalog[key] = { commands: [], slots: [], ...parsed.data };
  }
  return { catalog, problems };
}

/** The capability a command belongs to, or null when every device has it (SaveHome…). */
export const capabilityOfCommand = (catalog, command) =>
  Object.keys(catalog).find((key) => catalog[key].commands.includes(command)) ?? null;

/** The capability a telemetry slot belongs to, or null when every device has it (position…). */
export const capabilityOfSlot = (catalog, slot) =>
  Object.keys(catalog).find((key) => catalog[key].slots.includes(slot)) ?? null;

/**
 * The capabilities an interface implements: those with at least one of their slots declared, or
 * one of their commands resolved to an endpoint of the interface (commandRegistry, the same rule
 * that decides which commands a category exposes).
 * @returns {Set<string>}
 */
export function implementedBy(catalog, iface) {
  const slots = new Set(Object.keys(iface?.subscribers ?? {}));
  const { commands } = commandRegistry.resolveCommands(iface ?? {});
  const implemented = new Set();
  for (const [key, cap] of Object.entries(catalog)) {
    if (cap.slots.some((s) => slots.has(s)) || cap.commands.some((c) => commands.has(c))) implemented.add(key);
  }
  return implemented;
}

/**
 * The capabilities a device HAS: its model's (`'*'` = every one its interface implements; otherwise
 * the standard ones plus the optional ones the device enables, minus the ones it disables) that its
 * interface implements. `overrides` is `device.attributes.capabilities`: `{ rtk: true, gimbal: false }`.
 * @returns {Set<string>}
 */
export function effectiveCapabilities(catalog, model, iface, overrides = {}) {
  const implemented = implementedBy(catalog, iface);
  if (!model) return implemented; // a device whose model is unknown keeps what its interface offers
  if (model.capabilities === '*') return implemented;
  const declared = model.capabilities ?? {};
  const has = (key) => overrides[key] ?? declared[key] === 'standard';
  return new Set([...implemented].filter((key) => key in declared && has(key)));
}
