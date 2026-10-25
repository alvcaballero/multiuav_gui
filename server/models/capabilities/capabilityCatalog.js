// The capability catalog (config/devices/capabilities.yaml): what a vehicle can do, and which commands
// and telemetry slots implement it. Pure: the device models say which capabilities a vehicle has,
// the interfaces which ones they implement; this module only knows the vocabulary and the rules.
import { z } from 'zod';
import { CommandType } from '../../config/commandTypes.js';
import { KNOWN_SUBSCRIBER_KEYS } from '../../config/deviceMsgCatalog.js';
import { commandRegistry } from '../commands/commandRegistry.js';

// The three kinds of mission item a capability can own: mission_schema.yaml's `actions`, `params`
// and `waypoint_params`.
export const MISSION_ITEM_KINDS = Object.freeze(['actions', 'params', 'waypoint_params']);

const capabilitySchema = z
  .object({
    label: z.string().min(1),
    description: z.string().optional(),
    commands: z.array(z.enum(Object.values(CommandType))).optional(),
    slots: z.array(z.string()).optional(),
    mission: z
      .object(Object.fromEntries(MISSION_ITEM_KINDS.map((kind) => [kind, z.array(z.string()).optional()])))
      .strict()
      .optional(),
  })
  .strict();

/**
 * Validates the parsed catalog: known commands, slots and mission items, and each one in at most one
 * capability (a command in two would be gated by whichever is checked first).
 * @param {object} raw the parsed capabilities.yaml
 * @param {Record<string, Set<string>>} [missionKeys] mission_schema.yaml's keys per kind; without it
 *   mission items are not checked against the catalog
 * @returns {{ catalog: Record<string, object>, problems: {path: string[], message: string}[] }}
 */
export function buildCapabilityCatalog(raw, missionKeys = null) {
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
    const mission = Object.fromEntries(MISSION_ITEM_KINDS.map((kind) => [kind, parsed.data.mission?.[kind] ?? []]));
    for (const kind of MISSION_ITEM_KINDS) {
      for (const item of mission[kind]) {
        if (missionKeys && !missionKeys[kind]?.has(item)) {
          problems.push({ path: [key, 'mission', kind], message: `'${item}' is not in mission_schema.yaml ${kind}` });
        }
      }
    }
    const items = [
      ...(parsed.data.commands ?? []),
      ...(parsed.data.slots ?? []).map((s) => `slot:${s}`),
      ...MISSION_ITEM_KINDS.flatMap((kind) => mission[kind].map((item) => `${kind}:${item}`)),
    ];
    for (const item of items) {
      if (owner.has(item)) problems.push({ path: [key], message: `'${item}' already belongs to '${owner.get(item)}'` });
      else owner.set(item, key);
    }
    catalog[key] = { commands: [], slots: [], ...parsed.data, mission };
  }
  return { catalog, problems };
}

/** The capability a command belongs to, or null when every device has it (SaveHome…). */
export const capabilityOfCommand = (catalog, command) =>
  Object.keys(catalog).find((key) => catalog[key].commands.includes(command)) ?? null;

/** The capability a telemetry slot belongs to, or null when every device has it (position…). */
export const capabilityOfSlot = (catalog, slot) =>
  Object.keys(catalog).find((key) => catalog[key].slots.includes(slot)) ?? null;

/** The capability a mission item (`kind` of MISSION_ITEM_KINDS) belongs to, or null when every device has it. */
export const capabilityOfMissionItem = (catalog, kind, item) =>
  Object.keys(catalog).find((key) => catalog[key].mission?.[kind]?.includes(item)) ?? null;

/**
 * The capabilities an interface implements: those with at least one of their slots declared, one
 * of their commands resolved to an endpoint of the interface (commandRegistry, the same rule that
 * decides which commands a category exposes), or one of their mission items in its mission profile
 * (`camera` has neither commands nor slots: photos only exist in missions).
 * @param {Record<string, Set<string>>} [missionItems] the interface's mission profile keys per kind
 * @returns {Set<string>}
 */
export function implementedBy(catalog, iface, missionItems = {}) {
  const slots = new Set(Object.keys(iface?.subscribers ?? {}));
  const { commands } = commandRegistry.resolveCommands(iface ?? {});
  const inProfile = (cap) =>
    MISSION_ITEM_KINDS.some((kind) => cap.mission?.[kind]?.some((i) => missionItems[kind]?.has(i)));
  const implemented = new Set();
  for (const [key, cap] of Object.entries(catalog)) {
    if (cap.slots.some((s) => slots.has(s)) || cap.commands.some((c) => commands.has(c)) || inProfile(cap)) {
      implemented.add(key);
    }
  }
  return implemented;
}

/**
 * The capabilities a device HAS: its model's (`'*'` = every one its interface implements; otherwise
 * the standard ones plus the optional ones the device enables, minus the ones it disables) that its
 * interface implements. `overrides` is `device.attributes.capabilities`: `{ rtk: true, gimbal: false }`.
 * @returns {Set<string>}
 */
export function effectiveCapabilities(catalog, model, iface, overrides = {}, missionItems = {}) {
  const implemented = implementedBy(catalog, iface, missionItems);
  if (!model) return implemented; // a device whose model is unknown keeps what its interface offers
  if (model.capabilities === '*') return implemented;
  const declared = model.capabilities ?? {};
  const has = (key) => overrides[key] ?? declared[key] === 'standard';
  return new Set([...implemented].filter((key) => key in declared && has(key)));
}
