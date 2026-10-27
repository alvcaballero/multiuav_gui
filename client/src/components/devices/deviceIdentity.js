// The identity of a device as the device forms edit it, in the order its parts depend on each other:
// name → device model (what it IS) → interface / category (how it is talked to: one the model lists)
// → protocol (one that carries the interface's message format) → the protocol's own attributes
// (mavlink: system id). Mirrors the server: schemas/zod/devices.js (shape), deviceModelsModel.deviceProblem
// (the category must be one the model lists) and categoryModel.deviceProblem (the protocol must carry
// the category's message format, a mavlink device needs a system id). The server stays the authority:
// this only keeps the forms from sending what it would reject.

export const isMavlink = (protocol) => protocol === 'mavlink';

// The vehicle types of the server (config/status.js VEHICLE_TYPE), in the order the form offers them,
// with the text to show. Only a FILTER of the model list: a device's type is its model's, never
// stored. A type the server adds and this map lacks still shows, under its raw key, after these.
export const VEHICLE_TYPE_LABELS = {
  multirotor: 'Multirotor',
  vtol: 'VTOL',
  fixed_wing: 'Fixed wing',
  flapping_wing: 'Flapping wing (ornithopter)',
  ugv: 'Ground vehicle (UGV)',
  usv: 'Surface vessel (USV)',
};

/** The vehicle types that have at least one model (GET /api/device-models), in VEHICLE_TYPE_LABELS order. */
export function vehicleTypesOf(models) {
  const present = new Set((models ?? []).map((m) => m.vehicleType));
  const known = Object.keys(VEHICLE_TYPE_LABELS).filter((t) => present.has(t));
  return [...known, ...[...present].filter((t) => !(t in VEHICLE_TYPE_LABELS)).sort()];
}

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

// The radio on board (server ATTRIBUTE_GROUPS 'radio') and the onboard computer to ping ('companion'): all
// optional, empty means "none".
const IPV4 = /^(25[0-5]|2[0-4]\d|1?\d?\d)(\.(25[0-5]|2[0-4]\d|1?\d?\d)){3}$/;
const MAC = /^([0-9a-f]{2}[:-]){5}[0-9a-f]{2}$/i;

const ipv4Error = (ip) => (!ip || IPV4.test(ip.trim()) ? null : 'An IPv4 address like 10.20.0.64');
export const radioIpError = ipv4Error;
export const companionIpError = ipv4Error;
export const radioMacError = (mac) =>
  !mac || MAC.test(mac.trim()) ? null : 'A MAC like aa:bb:cc:dd:ee:ff';

/** True when the server would accept the identity fields of `item`. */
export function isDeviceIdentityValid(item) {
  if (!item?.name || !item.deviceModel || !item.category || !item.protocol) return false;
  const { radioIp, radioMac, companionIp } = item.attributes ?? {};
  if (radioIpError(radioIp) || radioMacError(radioMac) || companionIpError(companionIp)) {
    return false;
  }
  return !isMavlink(item.protocol) || systemIdError(item.attributes?.systemId) === null;
}

// `attributes.capabilities` belongs to the MODEL (which of its capabilities this unit has) and the radio and
// companion keys to the unit's hardware: none of them depends on the protocol. Every other key belongs to the
// protocol (mavlink: systemId).
const HARDWARE_KEYS = ['radioIp', 'radioMac', 'companionIp'];
const unitAttributes = (item) => {
  const attributes = item.attributes ?? {};
  const kept = {};
  if (attributes.capabilities) kept.capabilities = attributes.capabilities;
  for (const key of HARDWARE_KEYS) if (attributes[key]) kept[key] = attributes[key];
  return kept;
};

/**
 * The item after a protocol change: the protocol's attributes are kept only while it stays the same,
 * the capability switches and the radio always. {} (not undefined) so an edit clears them in the database too.
 */
export function withProtocol(item, protocol) {
  return {
    ...item,
    protocol,
    attributes: protocol === item.protocol ? (item.attributes ?? {}) : unitAttributes(item),
  };
}

/** The model's capabilities this unit has: its own switch, else on when the model says `standard`. */
export const hasCapability = (item, name, level) =>
  item.attributes?.capabilities?.[name] ?? level === 'standard';

/**
 * Whether the device being edited has capability `name` from its model and its own switches: what
 * decides if the form shows that capability's settings (camera_stream → the stream sources,
 * file_download → the file sources). `'*'` has them all; no model known (none chosen, unknown) shows
 * everything, as the server keeps everything for a device of an unknown model. Whether the interface
 * implements it is the server's call.
 */
export function capabilityEnabled(item, model, name) {
  if (!model || model.capabilities === '*') return true;
  const level = model.capabilities?.[name];
  return level != null && hasCapability(item, name, level);
}

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
 * protocol) is chosen again, and the capability switches were the old model's. The radio is the unit's.
 */
export function withModel(item, deviceModel) {
  if (deviceModel === item.deviceModel) return item;
  const cleared = withCategory(item, null, null);
  const attributes = unitAttributes(item);
  delete attributes.capabilities;
  return { ...cleared, deviceModel, attributes };
}
