import { minimal, standard, common, ardupilotmega } from 'node-mavlink';

// What the GCS knows about MAVLink messages: which classes it can decode, its own identity, and
// how a message is named in devices_msg.yaml. Pure: no sockets, no device lookups.
//
// node-mavlink 2.3.0 gotchas verified against PX4/ArduPilot SITL (docs/decisions/mavlink.md › Resultados F0):
//   - the header fields are `sysid` / `compid` / `msgid` (there is no `srcSystem`)
//   - AUTOPILOT_VERSION lives in `standard`, not `common`: every dialect below is needed
//   - in CommandLong/CommandInt the params (and x/y/z) are `_param1.._param7`; assigning
//     `param1` adds a stray property and 0 goes on the wire

export const REGISTRY = { ...minimal.REGISTRY, ...standard.REGISTRY, ...common.REGISTRY, ...ardupilotmega.REGISTRY };

// The standard identity of a ground station, the same one vehicleCommand.js uses over ROS.
export const GCS_SYSTEM_ID = 255;
export const GCS_COMPONENT_ID = 190; // MAV_COMP_ID_MISSIONPLANNER

// A category names a MAVLink message in devices_msg.yaml as `mavlink/<MESSAGE_NAME>`.
export const MSG_TYPE_PREFIX = 'mavlink/';
export const msgTypeOf = (messageName) => `${MSG_TYPE_PREFIX}${messageName}`;

/** The class that decodes a packet, or undefined for a message outside the loaded dialects. */
export const classOf = (packet) => REGISTRY[packet.header.msgid];

const BY_NAME = new Map(Object.values(REGISTRY).map((clazz) => [clazz.MSG_NAME, clazz]));

/** `mavlink/GLOBAL_POSITION_INT` → its class; undefined for another protocol's type or an unknown name. */
export function classOfMsgType(msgType) {
  if (typeof msgType !== 'string' || !msgType.startsWith(MSG_TYPE_PREFIX)) return undefined;
  return BY_NAME.get(msgType.slice(MSG_TYPE_PREFIX.length));
}

const MAV_COMP_ID_AUTOPILOT1 = 1;

/**
 * The ONLY place that builds a COMMAND_LONG: node-mavlink keeps the params in `_param1.._param7`,
 * and `msg.param1 = x` would add a stray property and send 0 (the ACK still says ACCEPTED).
 * @param {object} args
 * @param {number} args.targetSystem vehicle sysid; never 0 (broadcast)
 * @param {number} args.command MAV_CMD
 * @param {number[]} [args.params] up to 7, the missing ones are 0
 */
export function commandLong({ targetSystem, targetComponent = MAV_COMP_ID_AUTOPILOT1, command, params = [], confirmation = 0 }) {
  if (!(targetSystem >= 1 && targetSystem <= 254)) throw new Error(`COMMAND_LONG needs a vehicle sysid, got ${targetSystem}`);
  if (params.length > 7) throw new Error(`COMMAND_LONG takes 7 params, got ${params.length}`);
  const m = new common.CommandLong();
  m.targetSystem = targetSystem;
  m.targetComponent = targetComponent;
  m.command = command;
  m.confirmation = confirmation;
  [m._param1, m._param2, m._param3, m._param4, m._param5, m._param6, m._param7] = Array.from({ length: 7 }, (_, i) => params[i] ?? 0);
  return m;
}

/**
 * The ONLY place that builds a COMMAND_INT. Its x/y/z are `_param5/_param6/_param7` in node-mavlink
 * too: in F0 `msg.x = …` sent PX4 lat 0, lon 0, alt 0 and the ACK said ACCEPTED.
 * @param {object} args
 * @param {number} args.frame MAV_FRAME
 * @param {number[]} [args.params] param1..4
 * @param {number} args.lat degrees (→ int ×1e7)
 * @param {number} args.lon degrees (→ int ×1e7)
 * @param {number} args.alt metres, in `frame`
 */
export function commandInt({ targetSystem, targetComponent = MAV_COMP_ID_AUTOPILOT1, command, frame, params = [], lat, lon, alt }) {
  if (!(targetSystem >= 1 && targetSystem <= 254)) throw new Error(`COMMAND_INT needs a vehicle sysid, got ${targetSystem}`);
  if (params.length > 4) throw new Error(`COMMAND_INT takes 4 params, got ${params.length}`);
  if (![lat, lon, alt].every(Number.isFinite)) throw new Error(`COMMAND_INT ${command} needs a finite lat/lon/alt`);
  const m = new common.CommandInt();
  m.targetSystem = targetSystem;
  m.targetComponent = targetComponent;
  m.frame = frame;
  m.command = command;
  m.current = 0;
  m.autocontinue = 0;
  [m._param1, m._param2, m._param3, m._param4] = Array.from({ length: 4 }, (_, i) => params[i] ?? 0);
  m._param5 = Math.round(lat * 1e7);
  m._param6 = Math.round(lon * 1e7);
  m._param7 = alt;
  return m;
}

/** A command descriptor (models/commands/mavlinkCommand.js) as the message that goes on the wire. */
export function toCommandMessage(descriptor, targetSystem, { confirmation = 0 } = {}) {
  const { kind, command, params, targetComponent } = descriptor;
  if (kind === 'long') return commandLong({ targetSystem, targetComponent, command, params, confirmation });
  if (kind === 'int') {
    const { frame, lat, lon, alt } = descriptor;
    return commandInt({ targetSystem, targetComponent, command, frame, params, lat, lon, alt });
  }
  throw new Error(`unknown MAVLink command kind '${kind}'`);
}

const MAV_MISSION_TYPE_MISSION = 0;
const MAV_FRAME_GLOBAL_FRAMES = new Set([0, 3, 5, 6, 10, 11]); // x/y are lat/lon

/** MISSION_COUNT: opens an upload of `count` items (mission type MISSION). */
export function missionCount(targetSystem, count) {
  const m = new common.MissionCount();
  Object.assign(m, { targetSystem, targetComponent: MAV_COMP_ID_AUTOPILOT1, count, missionType: MAV_MISSION_TYPE_MISSION, opaqueId: 0 });
  return m;
}

/**
 * One item of a mission (missionEncodeMavlink.js) as MISSION_ITEM_INT. Unlike COMMAND_INT its
 * fields ARE `param1..4` / `x` / `y` / `z`. In a global frame x/y are lat/lon (→ int ×1e7);
 * otherwise they are the raw ints the command defines.
 */
export function missionItemInt(targetSystem, seq, item) {
  const global = MAV_FRAME_GLOBAL_FRAMES.has(item.frame);
  if (global && ![item.lat, item.lon, item.alt].every(Number.isFinite)) {
    throw new Error(`mission item ${seq} (MAV_CMD ${item.command}) needs a finite lat/lon/alt`);
  }
  const m = new common.MissionItemInt();
  const [param1 = 0, param2 = 0, param3 = 0, param4 = 0] = item.params ?? [];
  Object.assign(m, {
    targetSystem,
    targetComponent: MAV_COMP_ID_AUTOPILOT1,
    seq,
    frame: item.frame,
    command: item.command,
    current: item.current ? 1 : 0,
    autocontinue: 1,
    param1,
    param2,
    param3,
    param4,
    x: global ? Math.round(item.lat * 1e7) : item.x ?? 0,
    y: global ? Math.round(item.lon * 1e7) : item.y ?? 0,
    z: global ? item.alt : item.z ?? 0,
    missionType: MAV_MISSION_TYPE_MISSION,
  });
  return m;
}

export const missionResultName = (code) => common.MavMissionResult[code] ?? `MAV_MISSION_RESULT ${code}`;

/** MAV_CMD_SET_MESSAGE_INTERVAL: `hz` > 0 sets the rate, 0 restores the vehicle's default. */
export function setMessageInterval(targetSystem, clazz, hz) {
  const intervalUs = hz > 0 ? Math.round(1e6 / hz) : 0;
  return commandLong({ targetSystem, command: common.MavCmd.SET_MESSAGE_INTERVAL, params: [clazz.MSG_ID, intervalUs] });
}

/** @returns {object|null} the decoded message, or null when no dialect knows its id */
export function decode(packet) {
  const clazz = classOf(packet);
  return clazz ? packet.protocol.data(packet.payload, clazz) : null;
}

/** A heartbeat that comes from the autopilot, not from a camera/gimbal component or another GCS. */
export function isAutopilotHeartbeat(clazz, data) {
  return (
    clazz === minimal.Heartbeat &&
    data.autopilot !== minimal.MavAutopilot.INVALID &&
    data.type !== minimal.MavType.GCS
  );
}

export const isArmed = (heartbeat) => (heartbeat.baseMode & minimal.MavModeFlag.SAFETY_ARMED) !== 0;

export const autopilotName = (code) => minimal.MavAutopilot[code] ?? `#${code}`;

const plainValue = (value) => {
  if (typeof value === 'bigint') return Number.isSafeInteger(Number(value)) ? Number(value) : value.toString();
  if (Array.isArray(value)) return value.map(plainValue);
  return value;
};

/**
 * The message as a JSON-safe object keyed by the MAVLink spec field names (`param1`, `x`,
 * `target_system`), not node-mavlink's (`_param1`, `targetSystem`): it is what an operator reads
 * in the raw inspector next to the spec. BigInt (time_usec, capabilities) becomes a number when it
 * is safe, a string otherwise: the WS JSON.stringify throws on BigInt.
 */
export function toPlain(clazz, data) {
  return Object.fromEntries(clazz.FIELDS.map((f) => [f.source, plainValue(data[f.name])]));
}

/** The GCS heartbeat. PX4 refuses to arm (TEMPORARILY_REJECTED) until one has been received. */
export function gcsHeartbeat() {
  const hb = new minimal.Heartbeat();
  hb.type = minimal.MavType.GCS;
  hb.autopilot = minimal.MavAutopilot.INVALID;
  hb.baseMode = 0;
  hb.customMode = 0;
  hb.systemStatus = minimal.MavState.ACTIVE;
  hb.mavlinkVersion = 3;
  return hb;
}
