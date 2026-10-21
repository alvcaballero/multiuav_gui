import { EventEmitter } from 'node:events';
import { PassThrough } from 'node:stream';
import { MavLinkPacketSplitter, MavLinkPacketParser, MavLinkProtocolV2, minimal, common } from 'node-mavlink';

// MAVLink test fixtures. Lives under test/helpers so `node --test test/*.js` does not run it.
//
// Real packets: serialized by a "vehicle" protocol and parsed by node-mavlink's own pipeline, so a
// test never relies on a hand-made object that looks like a packet (the F0 traps were exactly that
// gap between the object and the wire). The pipeline emits synchronously inside write().

export function packetsOf(buffer) {
  const out = [];
  const input = new PassThrough();
  input.pipe(new MavLinkPacketSplitter()).pipe(new MavLinkPacketParser()).on('data', (p) => out.push(p));
  input.write(buffer);
  return out;
}

export const frame = (msg, sysid, compid = 1) => new MavLinkProtocolV2(sysid, compid).serialize(msg, 0);
export const packet = (msg, sysid, compid = 1) => packetsOf(frame(msg, sysid, compid))[0];

export function heartbeat({
  autopilot = minimal.MavAutopilot.PX4,
  type = minimal.MavType.QUADROTOR,
  baseMode = 29,
  customMode = 0,
  systemStatus = minimal.MavState.STANDBY,
} = {}) {
  const hb = new minimal.Heartbeat();
  Object.assign(hb, { type, autopilot, baseMode, customMode, systemStatus, mavlinkVersion: 3 });
  return hb;
}

export function globalPosition({ lat = 43.47, lon = -8.18, altMm = 100200, hdg = 9000, vx = 0, vy = 0, vz = 0 } = {}) {
  const m = new common.GlobalPositionInt();
  Object.assign(m, { timeBootMs: 1, lat: Math.round(lat * 1e7), lon: Math.round(lon * 1e7), alt: altMm, relativeAlt: 10, vx, vy, vz, hdg });
  return m;
}

export function commandAck(command, result, { targetSystem = 255, progress = 0 } = {}) {
  const m = new common.CommandAck();
  Object.assign(m, { command, result, progress, resultParam2: 0, targetSystem, targetComponent: 190 });
  return m;
}

export function missionRequestInt(seq, { targetSystem = 255 } = {}) {
  const m = new common.MissionRequestInt();
  Object.assign(m, { seq, targetSystem, targetComponent: 190, missionType: 0 });
  return m;
}

export function missionAck(type, { targetSystem = 255 } = {}) {
  const m = new common.MissionAck();
  Object.assign(m, { targetSystem, targetComponent: 190, type, missionType: 0, opaqueId: 0 });
  return m;
}

export function missionItemReached(seq) {
  const m = new common.MissionItemReached();
  m.seq = seq;
  return m;
}

export function statusText(severity, text, { id = 0, chunkSeq = 0 } = {}) {
  const m = new common.StatusText();
  Object.assign(m, { severity, text, id, chunkSeq });
  return m;
}

/**
 * A link that records what the server sends and lets a test play the vehicle with
 * `emit('packet')`. `onSend(msg, remote)` lets the vehicle answer (a COMMAND_ACK).
 */
export class FakeLink extends EventEmitter {
  constructor(id = 'udp:0.0.0.0:14550', kind = 'udp') {
    super();
    this.id = id;
    this.kind = kind;
    this.sent = [];
    this.onSend = null;
  }
  start() {}
  sendMessage(msg, remote) {
    this.sent.push({ msg, remote });
    this.onSend?.(msg, remote);
    return true;
  }
  close() {}
}
