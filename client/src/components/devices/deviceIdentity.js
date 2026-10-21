// The identity of a device (name, protocol, category, MAVLink system id, IP) as the device forms
// edit it. Mirrors the server: schemas/zod/devices.js (shape) and categoryModel.deviceProblem
// (the category's message format must be one the protocol carries, a mavlink device needs a
// system id). The server stays the authority: this only keeps the forms from sending what it
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
  if (!item?.name || !item.protocol || !item.category || !item.ip) return false;
  return !isMavlink(item.protocol) || systemIdError(item.systemId) === null;
}

/**
 * The item after a protocol change: a category in a message format the new protocol does not carry
 * would leave the device deaf, so it is cleared (the category list reloads for the new protocol);
 * a system id only exists on mavlink. null (not undefined) so an edit clears it in the database too.
 */
export function withProtocol(item, protocol) {
  const sameFormat = messageFormat(protocol) === messageFormat(item.protocol);
  return {
    ...item,
    protocol,
    category: sameFormat ? item.category : null,
    systemId: isMavlink(protocol) ? (item.systemId ?? null) : null,
  };
}
