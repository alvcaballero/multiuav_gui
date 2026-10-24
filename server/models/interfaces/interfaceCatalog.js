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

const prefixed = (key, problems) => problems.map(({ path, message }) => ({ path: [key, ...path], message }));

/**
 * Validates every interface of `raw` (key → file contents). An interface with any problem is
 * LEFT OUT of `catalog`; the others are not affected.
 * @returns {{ catalog: Record<string, object>, problems: {path: (string|number)[], message: string}[] }}
 */
export function buildCatalog(raw) {
  const catalog = {};
  const problems = [];
  for (const [key, def] of Object.entries(raw)) {
    const own = interfaceShapeProblems(def);
    if (own.length === 0) own.push(...resolvedProblems(def));
    if (own.length) problems.push(...prefixed(key, own));
    else catalog[key] = def;
  }
  return { catalog, problems };
}

/** Reads `dir` and builds its catalog; file problems (bad name, bad YAML) come first. */
export function loadInterfaces(dir) {
  const { raw, problems: fileProblems } = readInterfaceDir(dir);
  const { catalog, problems } = buildCatalog(raw);
  return { raw, catalog, problems: [...fileProblems, ...problems] };
}

export const formatProblem = ({ path, message }) => `${path.join('.')}: ${message}`;
