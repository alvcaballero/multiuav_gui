// The interface catalog (what devices_msg.yaml used to be): read, validated and resolved, with no
// side effects, so the boot, the API (before writing a file) and the tests run the SAME checks.
// What a slot or a command means (decoders, command registry) is checked by category.js on top.
import { interfaceShapeProblems } from '../../schemas/zod/interface.js';
import { readInterfaceDir } from './interfaceStore.js';

export const ENDPOINT_BLOCKS = ['subscribers', 'publishers', 'services', 'actions'];

// The MESSAGE FORMAT an interface speaks — not a protocol. The protocols are four (ros, mqtt,
// robofleet, mavlink), each with its own gateway and its own software on the robot; the device's
// `protocol` picks the gateway. What an interface declares is the shape of its messages, and a
// protocol can only carry one shape:
//   - MAVLINK: `mavlink/<MSG>` messages → only the mavlink protocol.
//   - ROS_MSG: ROS-shaped messages (sensor_msgs/NavSatFix…) → ros (rosbridge JSON), mqtt (the same
//     JSON) and robofleet (ROS messages packed in Flatbuffers).
// DERIVED from the interface's messageTypes, nothing declared.
export const MESSAGE_FORMAT = Object.freeze({ MAVLINK: 'mavlink', ROS_MSG: 'ros_msg' });
const MAVLINK_PREFIX = 'mavlink/';
const isMavlinkType = (msgType) => String(msgType ?? '').startsWith(MAVLINK_PREFIX);

export function messageFormatOf(def) {
  const speaksMavlink = ENDPOINT_BLOCKS.some((block) =>
    Object.values(def?.[block] ?? {}).some((entry) => isMavlinkType(entry?.messageType))
  );
  return speaksMavlink ? MESSAGE_FORMAT.MAVLINK : MESSAGE_FORMAT.ROS_MSG;
}

/** Every endpoint entry of a resolved interface, heartbeat included: [path, entry]. */
function* entriesOf(def) {
  for (const block of ENDPOINT_BLOCKS) {
    for (const [key, entry] of Object.entries(def?.[block] ?? {})) yield [[block, key], entry];
  }
  if (def?.heartbeat) yield [['heartbeat'], def.heartbeat];
}

// A resolved interface speaks ONE message format, every entry names its messageType, and `name`
// (the ROS topic/service/action) is required for ROS messages and meaningless for MAVLink.
function resolvedProblems(def) {
  const problems = [];
  const format = messageFormatOf(def);
  for (const [path, entry] of entriesOf(def)) {
    if (!entry?.messageType) {
      problems.push({ path, message: 'missing messageType' });
      continue;
    }
    if (isMavlinkType(entry.messageType) !== (format === MESSAGE_FORMAT.MAVLINK)) {
      problems.push({ path, message: `mixes message formats: '${entry.messageType}' in a ${format} interface` });
    } else if (format === MESSAGE_FORMAT.ROS_MSG && !entry.name) {
      problems.push({ path, message: 'missing name (the ROS topic/service/action)' });
    } else if (format === MESSAGE_FORMAT.MAVLINK && entry.name) {
      problems.push({ path, message: 'a MAVLink entry has no name (there are no topics)' });
    }
  }
  return problems;
}

// What a concrete interface must say about itself (an abstract base may leave it to its children).
function identityProblems(def) {
  const problems = [];
  if (!def.firmwares) problems.push({ path: ['firmwares'], message: 'missing firmwares' });
  if (!def.vehicleTypes) problems.push({ path: ['vehicleTypes'], message: 'missing vehicleTypes' });
  return problems;
}

const prefixed = (key, problems) => problems.map(({ path, message }) => ({ path: [key, ...path], message }));

// Blocks that are maps of entries: a child merges entry by entry (a `null` entry removes the
// inherited one). Everything else (firmwares, vehicleTypes, mission…) the child just replaces.
const ENTRY_BLOCKS = [...ENDPOINT_BLOCKS, 'commands'];

function mergeEntries(parentBlock = {}, childBlock = {}) {
  const merged = { ...parentBlock };
  for (const [key, entry] of Object.entries(childBlock)) {
    if (entry === null) delete merged[key];
    else merged[key] = { ...(merged[key] ?? {}), ...entry };
  }
  return merged;
}

/** `child` over its resolved `parent`; `abstract` is never inherited. */
export function mergeInterface(parent, child) {
  const { abstract, ...out } = { ...parent, ...child };
  for (const block of ENTRY_BLOCKS) {
    if (block in child) out[block] = mergeEntries(parent[block], child[block]);
  }
  if (child.heartbeat === null) delete out.heartbeat;
  else if (child.heartbeat) out.heartbeat = { ...(parent.heartbeat ?? {}), ...child.heartbeat };
  if (child.abstract) out.abstract = true;
  return out;
}

const hasNullEntries = (def) =>
  def.heartbeat === null || ENTRY_BLOCKS.some((block) => Object.values(def[block] ?? {}).includes(null));

/**
 * Validates and resolves every interface of `raw` (key → file contents), in two passes: shape of
 * each file, then `extends` (so the order of the files does not matter). An interface with any
 * problem is LEFT OUT, and so are its children (their problem says why); the others are not
 * affected. Abstract interfaces are only bases: they are in `abstract`, never in `catalog`.
 * @returns {{
 *   catalog: Record<string, object>,
 *   abstract: Record<string, object>,
 *   problems: {path: (string|number)[], message: string}[],
 * }}
 */
export function buildCatalog(raw) {
  const problems = [];
  const shaped = {};
  for (const [key, def] of Object.entries(raw)) {
    const own = interfaceShapeProblems(def);
    if (own.length === 0 && !def.extends && hasNullEntries(def)) {
      own.push({ path: [], message: 'a null entry removes an inherited one, and this interface extends nothing' });
    }
    if (own.length) problems.push(...prefixed(key, own));
    else shaped[key] = def ?? {};
  }

  const resolved = {};
  const failed = new Set(Object.keys(raw).filter((key) => !shaped[key]));
  const resolve = (key, chain) => {
    if (resolved[key] || failed.has(key)) return resolved[key];
    const def = shaped[key];
    const fail = (message) => {
      problems.push({ path: [key, 'extends'], message });
      failed.add(key);
    };
    if (!def.extends) return (resolved[key] = mergeInterface({}, def));
    const parentKey = def.extends;
    if (parentKey === key || chain.includes(parentKey)) return fail(`cycle: ${[...chain, key, parentKey].join(' → ')}`);
    if (!(parentKey in raw)) return fail(`extends unknown interface '${parentKey}'`);
    const parent = resolve(parentKey, [...chain, key]);
    if (!parent) return fail(`not loaded because its parent '${parentKey}' is invalid`);
    return (resolved[key] = mergeInterface(parent, def));
  };
  for (const key of Object.keys(shaped)) resolve(key, []);

  const catalog = {};
  const abstract = {};
  for (const [key, def] of Object.entries(resolved)) {
    if (def.abstract) {
      abstract[key] = def;
      continue;
    }
    const own = [...identityProblems(def), ...resolvedProblems(def)];
    if (own.length) problems.push(...prefixed(key, own));
    else catalog[key] = def;
  }
  return { catalog, abstract, problems };
}

/** The interface at the top of `key`'s `extends` chain (itself when it extends nothing). */
export function rootOf(raw, key) {
  const seen = new Set();
  let current = key;
  while (raw[current]?.extends && !seen.has(current)) {
    seen.add(current);
    current = raw[current].extends;
  }
  return current;
}

/** Reads `dir` and builds its catalog; file problems (bad name, bad YAML) come first. */
export function loadInterfaces(dir) {
  const { raw, problems: fileProblems } = readInterfaceDir(dir);
  const { catalog, abstract, problems } = buildCatalog(raw);
  return { raw, catalog, abstract, problems: [...fileProblems, ...problems] };
}

export const formatProblem = ({ path, message }) => `${path.join('.')}: ${message}`;
