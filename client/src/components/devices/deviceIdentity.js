// The identity of a device as the device forms edit it, in the order its parts depend on each other:
// name → device model (what it IS) → interface / category (how it is talked to: one the model lists)
// → protocol (one that carries the interface's message format) → the protocol's own attributes
// (mavlink: system id). Mirrors the server: schemas/zod/devices.js (shape), deviceModelsModel.deviceProblem
// (the category must be one the model lists) and categoryModel.deviceProblem (the protocol must carry
// the category's message format, a mavlink device needs a system id). The server stays the authority:
// this only keeps the forms from sending what it would reject.

export const isMavlink = (protocol) => protocol === 'mavlink';

// The message format a protocol carries (server: MESSAGE_FORMAT). The protocols are four, each with
// its own gateway; ros, mqtt and robofleet carry the same ROS-shaped messages, mavlink carries MAVLink.
export const messageFormat = (protocol) => (isMavlink(protocol) ? 'mavlink' : 'ros_msg');

/** The protocols (GET /api/server/protocol) that can carry an interface of this message format. */
export const protocolsFor = (protocols, format) =>
  (protocols ?? []).filter((p) => messageFormat(p.name) === format);

// 0 is broadcast and 255 is the GCS itself: neither names a vehicle.
const SYSTEM_ID_MIN = 1;
const SYSTEM_ID_MAX = 254;

export const systemIdError = (systemId) =>
  Number.isInteger(systemId) && systemId >= SYSTEM_ID_MIN && systemId <= SYSTEM_ID_MAX
    ? null
    : `Required: an integer between ${SYSTEM_ID_MIN} and ${SYSTEM_ID_MAX}`;

/** True when the server would accept the identity fields of `item`. */
export function isDeviceIdentityValid(item) {
  if (!item?.name || !item.deviceModel || !item.category || !item.protocol) return false;
  return !isMavlink(item.protocol) || systemIdError(item.attributes?.systemId) === null;
}

// `attributes.capabilities` belongs to the MODEL (which of its capabilities this unit has), every
// other key to the protocol (mavlink: systemId).
const capabilityOverrides = (item) =>
  item.attributes?.capabilities ? { capabilities: item.attributes.capabilities } : {};

/**
 * The item after a protocol change: the protocol's attributes are kept only while it stays the same,
 * the capability switches always. {} (not undefined) so an edit clears them in the database too.
 */
export function withProtocol(item, protocol) {
  return {
    ...item,
    protocol,
    attributes: protocol === item.protocol ? (item.attributes ?? {}) : capabilityOverrides(item),
  };
}

/** The model's capabilities this unit has: its own switch, else on when the model says `standard`. */
export const hasCapability = (item, name, level) =>
  item.attributes?.capabilities?.[name] ?? level === 'standard';

/**
 * The item with capability `name` switched to `on`. A switch that matches the model's default is
 * dropped: only what makes this unit differ from its model is stored.
 */
export function withCapability(item, name, level, on) {
  const capabilities = { ...item.attributes?.capabilities };
  if (on === (level === 'standard')) delete capabilities[name];
  else capabilities[name] = on;
  const rest = { ...item.attributes };
  delete rest.capabilities;
  return {
    ...item,
    attributes: Object.keys(capabilities).length ? { ...rest, capabilities } : rest,
  };
}

/**
 * The item after a category change. The protocol stays if it carries the new category's message
 * format (`format`, from GET /api/category?details=true), else it is cleared with its attributes:
 * a mavlink device on a ROS interface would be deaf.
 */
export function withCategory(item, category, format) {
  if (category === item.category) return item;
  const keepsProtocol = item.protocol && format && messageFormat(item.protocol) === format;
  return keepsProtocol ? { ...item, category } : { ...withProtocol(item, null), category };
}

/**
 * The item after a model change: its interfaces are another list, so the category (and with it the
 * protocol) is chosen again, and the capability switches were the old model's.
 */
export function withModel(item, deviceModel) {
  if (deviceModel === item.deviceModel) return item;
  const cleared = withCategory(item, null, null);
  return { ...cleared, deviceModel, attributes: {} };
}
