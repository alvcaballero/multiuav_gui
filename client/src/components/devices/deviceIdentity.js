// The identity of a device (name, protocol, category, device model, the protocol's own attributes) as
// the device forms edit it. Mirrors the server: schemas/zod/devices.js (shape), categoryModel.deviceProblem
// (the category's message format must be one the protocol carries, a mavlink device needs a
// system id) and deviceModelsModel.deviceProblem (the model must be one the category can drive: the
// list only offers those). The server stays the authority: this only keeps the forms from sending what it
// would reject.

export const isMavlink = (protocol) => protocol === 'mavlink';

// The message format a protocol carries (server: MESSAGE_FORMAT). The protocols are four, each with
// its own gateway; ros, mqtt and robofleet carry the same ROS-shaped messages, so they offer the
// same categories, and mavlink carries MAVLink.
export const messageFormat = (protocol) => (isMavlink(protocol) ? 'mavlink' : 'ros_msg');

// 0 is broadcast and 255 is the GCS itself: neither names a vehicle.
const SYSTEM_ID_MIN = 1;
const SYSTEM_ID_MAX = 254;

export const systemIdError = (systemId) =>
  Number.isInteger(systemId) && systemId >= SYSTEM_ID_MIN && systemId <= SYSTEM_ID_MAX
    ? null
    : `Required: an integer between ${SYSTEM_ID_MIN} and ${SYSTEM_ID_MAX}`;

/** True when the server would accept the identity fields of `item`. */
export function isDeviceIdentityValid(item) {
  if (!item?.name || !item.protocol || !item.category) return false;
  return !isMavlink(item.protocol) || systemIdError(item.attributes?.systemId) === null;
}

/**
 * The item after a protocol change: a category in a message format the new protocol does not carry
 * would leave the device deaf, so it is cleared (the category list reloads for the new protocol);
 * `attributes` belong to the protocol (mavlink: systemId), so they are kept only while it stays the
 * same. {} (not undefined) so an edit clears them in the database too.
 */
export function withProtocol(item, protocol) {
  const sameFormat = messageFormat(protocol) === messageFormat(item.protocol);
  return {
    ...(sameFormat ? item : withCategory(item, null)),
    protocol,
    attributes: protocol === item.protocol ? (item.attributes ?? {}) : {},
  };
}

/**
 * The item after a category change: the device model is cleared, so the model list (only the models
 * the new category can drive, GET /api/device-models?category=) auto-selects that category's default.
 * Keeping it could send a model the server refuses for the new category (a DJI M300 on px4_ros2).
 */
export function withCategory(item, category) {
  return category === item.category ? item : { ...item, category, deviceModel: null };
}
