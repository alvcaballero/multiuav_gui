import { test, describe, after } from 'node:test';
import assert from 'node:assert/strict';
import dgram from 'node:dgram';
import { MavLinkProtocolV2, minimal, common } from 'node-mavlink';
import { MavlinkServer, slotOf } from '../models/mavlink/MavlinkServer.js';
import { UdpLink, parseLinkSpec, parseLinkSpecs } from '../models/mavlink/links.js';
import { REGISTRY, toPlain, decode, gcsHeartbeat } from '../models/mavlink/mavlinkMessages.js';
import { createMavlinkGateway } from '../models/gateways/mavlinkGateway.js';
import { validateDevice } from '../schemas/zod/devices.js';
import { commandLong, setMessageInterval, classOfMsgType } from '../models/mavlink/mavlinkMessages.js';
import { decodeMsg } from '../models/msgDecode.js';
import { categoryModel } from '../models/category.js';
import { ARM_STATE, FLIGHT_MODE, FLIGHT_PHASE, COMMAND_RESULT, VEHICLE_TYPE } from '../config/status.js';
import { vehicleTypeOfMavType } from '../models/vehicleStates.js';
import {
  packetsOf,
  frame,
  packet,
  heartbeat,
  globalPosition,
  FakeLink,
  commandAck,
  statusText,
  missionRequestInt,
  missionAck,
  missionItemReached,
} from './helpers/mavlinkFixtures.js';
import { encodeMavlinkMission, MAV_CMD_MISSION, MISSION_FINISH } from '../models/mission/missionEncodeMavlink.js';
import { MAVLINK_MISSION } from '../models/mission/missionSymbols.js';
import { missionItemInt } from '../models/mavlink/mavlinkMessages.js';
import { buildMavlinkCommand, MAVLINK_COMMAND, MAV_CMD } from '../models/commands/mavlinkCommand.js';
import { commandRegistry } from '../models/commands/commandRegistry.js';
import { contentsOf } from '../models/commands/commandEncoders.js';
import { toCommandMessage, commandInt } from '../models/mavlink/mavlinkMessages.js';
import { CommandType } from '../config/commandTypes.js';

const DEVICE = { id: 7, name: 'px4_1', category: 'px4_mavlink', protocol: 'mavlink', attributes: { systemId: 1 } };
const CATEGORY = { subscribers: { position_global: { messageType: 'mavlink/GLOBAL_POSITION_INT' } } };
const PX4_REMOTE = { address: '127.0.0.1', port: 18570 };

const servers = [];
after(() => servers.forEach((s) => s.close()));

function setup({ devicesBySysid = { 1: [DEVICE] }, category = CATEGORY, links = [new FakeLink()], serverOpts = {} } = {}) {
  let clock = 1000;
  const taps = [];
  const lookups = [];
  const updates = [];
  const finished = [];
  const queue = [...links];
  const server = new MavlinkServer(
    links.map((l) => ({ kind: l.kind, id: l.id })),
    {
      createLink: () => queue.shift(),
      now: () => clock,
      presenceTimeoutMs: 5000,
      ...serverOpts,
      deps: {
        devices: { getBySystemId: async (s) => (lookups.push(s), devicesBySysid[s] ?? []) },
        positions: { updatePosition: (p) => updates.push(p) },
        missions: { deviceFinishMission: (d) => finished.push(d) },
        categories: { getCategory: () => category },
        rawTap: { publish: (m) => taps.push(m) },
        telemetryWatchdog: { record() {} },
      },
    }
  );
  servers.push(server);
  const [link] = links;
  return {
    server,
    link,
    links,
    taps,
    lookups,
    updates,
    finished,
    now: () => clock,
    advance: (ms) => (clock += ms),
    feed: (msg, sysid, { compid = 1, remote = PX4_REMOTE, via = link } = {}) => via.emit('packet', packet(msg, sysid, compid), remote),
  };
}

const tick = () => new Promise((r) => setImmediate(r));

describe('mavlink links: spec', () => {
  test('parses udp and tcp, rejects serial (not yet) and garbage', () => {
    assert.deepEqual(parseLinkSpec('udp:0.0.0.0:14550'), { kind: 'udp', host: '0.0.0.0', port: 14550, id: 'udp:0.0.0.0:14550' });
    assert.equal(parseLinkSpec('tcp:127.0.0.1:5760').kind, 'tcp');
    assert.throws(() => parseLinkSpec('serial:/dev/ttyUSB0:57600'), /not supported yet/);
    assert.throws(() => parseLinkSpec('udp:14550'), /invalid MAVLink link/);
    assert.equal(parseLinkSpecs('udp:0.0.0.0:14550, tcp:127.0.0.1:5760').length, 2);
  });
});

describe('mavlink messages', () => {
  test('the GCS heartbeat goes out as a GCS with no autopilot', () => {
    const p = packetsOf(new MavLinkProtocolV2(255, 190).serialize(gcsHeartbeat(), 0))[0];
    const hb = decode(p);
    assert.equal(p.header.sysid, 255);
    assert.equal(p.header.compid, 190);
    assert.equal(hb.type, minimal.MavType.GCS);
    assert.equal(hb.autopilot, minimal.MavAutopilot.INVALID);
  });

  // The trap of F0: node-mavlink keeps the params in `_param1..7` (also x/y/z in CommandInt).
  test('toPlain keys a message by the spec field names, as it went on the wire', () => {
    const m = new common.CommandInt();
    Object.assign(m, { targetSystem: 1, targetComponent: 1, frame: 0, command: 22, current: 0, autocontinue: 0 });
    [m._param1, m._param2, m._param3, m._param4, m._param5, m._param6, m._param7] = [0, 0, 0, 0, 434726046, -81788478, 110.5];
    const p = packet(m, 255, 190);
    const plain = toPlain(REGISTRY[p.header.msgid], decode(p));
    assert.equal(plain.x, 434726046);
    assert.equal(plain.y, -81788478);
    assert.ok(Math.abs(plain.z - 110.5) < 1e-4);
    assert.equal(plain.target_system, 1);
    assert.equal('_param5' in plain, false);
  });

  test('toPlain turns BigInt into something JSON.stringify accepts', () => {
    const m = new common.SystemTime();
    Object.assign(m, { timeUnixUsec: 1761000000000000n, timeBootMs: 5 });
    const plain = toPlain(common.SystemTime, decode(packet(m, 1)));
    assert.equal(plain.time_unix_usec, 1761000000000000);
    assert.doesNotThrow(() => JSON.stringify(plain));
  });
});

describe('mavlink UdpLink (real sockets on loopback)', () => {
  test('tells two vehicles on one port apart by remote, and answers each one', async () => {
    const link = new UdpLink({ kind: 'udp', host: '127.0.0.1', port: 0, id: 'udp:test' });
    await link.start();
    const got = [];
    link.on('packet', (p, remote) => got.push({ sysid: p.header.sysid, remote }));

    const px4 = dgram.createSocket('udp4');
    const ardu = dgram.createSocket('udp4');
    await Promise.all([px4, ardu].map((s) => new Promise((r) => s.bind(0, '127.0.0.1', r))));
    const fromGcs = new Promise((resolve) => px4.once('message', (buf) => resolve(packetsOf(buf)[0])));

    px4.send(frame(heartbeat(), 1), link.port, '127.0.0.1');
    ardu.send(frame(heartbeat({ autopilot: minimal.MavAutopilot.ARDUPILOTMEGA }), 2), link.port, '127.0.0.1');
    await new Promise((r) => setTimeout(r, 50));

    const bySysid = Object.fromEntries(got.map((g) => [g.sysid, g.remote.port]));
    assert.equal(bySysid[1], px4.address().port);
    assert.equal(bySysid[2], ardu.address().port);

    link.sendMessage(gcsHeartbeat(), got.find((g) => g.sysid === 1).remote);
    const reply = await fromGcs;
    assert.equal(reply.header.sysid, 255);

    px4.close();
    ardu.close();
    link.close();
  });
});

describe('MavlinkServer: discovery and presence', () => {
  test('an autopilot heartbeat brings the vehicle online and remembers its route', () => {
    const { server, feed, link } = setup();
    const online = [];
    server.on('online', (v) => online.push(v.sysid));
    assert.equal(server.presenceOf(1), 'unknown');
    feed(heartbeat(), 1);
    assert.equal(server.presenceOf(1), 'online');
    assert.deepEqual(online, [1]);
    assert.equal(server.getVehicle(1).link, link);
    assert.deepEqual(server.getVehicle(1).remote, PX4_REMOTE);
  });

  test('heartbeats of a component, of a GCS and of sysid 255 do not make a vehicle', () => {
    const { server, feed } = setup();
    feed(heartbeat({ autopilot: minimal.MavAutopilot.INVALID, type: minimal.MavType.CAMERA }), 3, { compid: 100 });
    feed(heartbeat({ autopilot: minimal.MavAutopilot.INVALID, type: minimal.MavType.GCS }), 4);
    feed(heartbeat(), 255);
    assert.equal(server.vehicles.size, 0);
  });

  test('a vehicle without heartbeat for the timeout goes offline, and comes back', () => {
    const { server, feed, advance } = setup();
    const events = [];
    server.on('online', () => events.push('online'));
    server.on('offline', () => events.push('offline'));
    feed(heartbeat(), 1);
    advance(4000);
    server.presenceTick();
    assert.equal(server.presenceOf(1), 'online');
    advance(2000);
    server.presenceTick();
    assert.equal(server.presenceOf(1), 'offline');
    feed(heartbeat(), 1);
    assert.deepEqual(events, ['online', 'offline', 'online']);
  });

  test('a live sysid seen from a second route keeps the first one (duplicated MAV_SYS_ID)', () => {
    const { server, feed } = setup();
    feed(heartbeat(), 1);
    feed(heartbeat({ autopilot: minimal.MavAutopilot.ARDUPILOTMEGA }), 1, { remote: { address: '127.0.0.1', port: 53383 } });
    assert.deepEqual(server.getVehicle(1).remote, PX4_REMOTE);
    assert.equal(server.getVehicle(1).autopilot, minimal.MavAutopilot.PX4);
  });

  test('once offline, the same sysid from a new port takes the new route (ArduPilot restart)', () => {
    const { server, feed, advance } = setup();
    feed(heartbeat(), 1);
    advance(6000);
    server.presenceTick();
    const newRemote = { address: '127.0.0.1', port: 40000 };
    feed(heartbeat(), 1, { remote: newRemote });
    assert.equal(server.presenceOf(1), 'online');
    assert.deepEqual(server.getVehicle(1).remote, newRemote);
  });
});

describe('MavlinkServer: GCS heartbeat', () => {
  test('goes to every live vehicle on its link, once per route, and marks when it started', () => {
    const { server, feed, link } = setup();
    feed(heartbeat(), 1);
    feed(heartbeat({ autopilot: minimal.MavAutopilot.ARDUPILOTMEGA }), 2, { remote: { address: '127.0.0.1', port: 53383 } });
    server.heartbeatTick(2000);
    assert.equal(link.sent.length, 2);
    assert.ok(link.sent.every(({ msg }) => msg.type === minimal.MavType.GCS));
    assert.deepEqual(link.sent.map((s) => s.remote.port).sort(), [18570, 53383]);
    assert.equal(server.getVehicle(1).gcsHeartbeatSince, 2000);
    server.heartbeatTick(3000);
    assert.equal(server.getVehicle(1).gcsHeartbeatSince, 2000, 'the start of the GCS heartbeat does not move');
  });

  test('stops for an offline vehicle, and restarts its clock when it comes back', () => {
    const { server, feed, link, advance } = setup();
    feed(heartbeat(), 1);
    server.heartbeatTick(1000);
    advance(6000);
    server.presenceTick();
    link.sent.length = 0;
    server.heartbeatTick(7000);
    assert.equal(link.sent.length, 0);
    assert.equal(server.getVehicle(1).gcsHeartbeatSince, null);
  });
});

describe('MavlinkServer: raw inspector', () => {
  // The plain form is built anyway for the decoder (position), so the tap gets that same object.
  test('a message declared by the category is tapped under its slot, by spec names', async () => {
    const { feed, taps } = setup();
    feed(heartbeat(), 1);
    feed(globalPosition(), 1);
    await tick();
    const tap = taps.find((t) => t.slot === 'position_global');
    assert.ok(tap, 'tapped');
    assert.equal(tap.deviceId, 7);
    assert.equal(tap.transport, 'mavlink');
    assert.equal(tap.raw.lat, 434700000);
    assert.equal(tap.raw.relative_alt, 10);
  });

  test('a message the category does not declare is not tapped', async () => {
    const { feed, taps } = setup();
    feed(heartbeat(), 1);
    feed(globalPosition(), 1);
    const att = new common.Attitude();
    Object.assign(att, { timeBootMs: 1, roll: 0, pitch: 0, yaw: 0, rollspeed: 0, pitchspeed: 0, yawspeed: 0 });
    feed(att, 1);
    await tick();
    assert.deepEqual(taps.map((t) => t.slot), ['position_global']);
  });

  test('nothing from a sysid before its autopilot heartbeat, nor from an unknown sysid', async () => {
    const { feed, taps } = setup();
    feed(globalPosition(), 1); // no heartbeat yet
    feed(heartbeat(), 9);
    feed(globalPosition(), 9); // no device has systemId 9
    await tick();
    assert.equal(taps.length, 0);
  });

  test('a sysid shared by two devices is ignored instead of guessed', async () => {
    const { feed, taps } = setup({ devicesBySysid: { 1: [DEVICE, { ...DEVICE, id: 8, name: 'px4_2' }] } });
    feed(heartbeat(), 1);
    feed(globalPosition(), 1);
    await tick();
    assert.equal(taps.length, 0);
  });

  test('the sysid -> device lookup is cached (telemetry is tens of Hz)', async () => {
    const { feed, lookups } = setup();
    feed(heartbeat(), 1);
    for (let i = 0; i < 5; i++) feed(globalPosition(), 1);
    await tick();
    await tick();
    assert.equal(lookups.length, 1, 'packets that arrive while the query is in flight reuse it');
  });

  test('a device created while its vehicle is online shows up in seconds, not after the 30 s cache', async () => {
    const devicesBySysid = {};
    const { feed, taps, advance } = setup({ devicesBySysid });
    feed(heartbeat(), 1);
    feed(globalPosition(), 1);
    await tick();
    assert.equal(taps.length, 0, 'no device yet');
    devicesBySysid[1] = [DEVICE];
    advance(3500);
    feed(globalPosition(), 1);
    await tick();
    await tick();
    assert.equal(taps.length, 1);
  });

  test('slotOf finds the subscriber by messageType', () => {
    assert.equal(slotOf(CATEGORY.subscribers, 'mavlink/GLOBAL_POSITION_INT'), 'position_global');
    assert.equal(slotOf(CATEGORY.subscribers, 'mavlink/ATTITUDE'), undefined);
    assert.equal(slotOf(undefined, 'mavlink/ATTITUDE'), undefined);
  });
});

describe('mavlink gateway', () => {
  test('disabled transport answers an error instead of throwing', async () => {
    const r = await createMavlinkGateway({ getServer: () => null }).sendCommand({ device: DEVICE, endpoint: { key: 'x' } });
    assert.equal(r.state, 'error');
    assert.match(r.msg, /disabled/);
  });

  test('an endpoint that is not a MAVLink command dialect (missions, F4): a warning, never a silent success', async () => {
    const { server } = setup();
    const r = await createMavlinkGateway({ getServer: () => server }).sendCommand({ device: DEVICE, endpoint: { key: 'vehicle_command' } });
    assert.equal(r.state, 'warning');
  });
});

describe('device schema: mavlink attributes.systemId', () => {
  const base = { name: 'px4_1', category: 'px4_mavlink', protocol: 'mavlink' };
  test('a mavlink device needs a systemId', () => {
    const r = validateDevice(base);
    assert.equal(r.success, false);
    assert.deepEqual(r.error.issues[0].path, ['attributes', 'systemId']);
  });
  test('accepts 1..254, rejects broadcast 0 and the GCS 255', () => {
    assert.equal(validateDevice({ ...base, attributes: { systemId: 1 } }).success, true);
    assert.equal(validateDevice({ ...base, attributes: { systemId: '254' } }).success, true);
    assert.equal(validateDevice({ ...base, attributes: { systemId: 0 } }).success, false);
    assert.equal(validateDevice({ ...base, attributes: { systemId: 255 } }).success, false);
  });
  test('other protocols do not need it', () => {
    assert.equal(validateDevice({ ...base, protocol: 'ros' }).success, true);
  });
});

// ─── F2: builders, decoders, streams ──────────────────────────────────────────

/** What a decoder receives: the message after the wire, as MavlinkServer hands it over. */
const plainOf = (msg, sysid = 1, compid = 1) => {
  const p = packet(msg, sysid, compid);
  return toPlain(REGISTRY[p.header.msgid], decode(p));
};
const decodeAs = (slot, name, msg) => decodeMsg({ msg: plainOf(msg), deviceId: 7, type: slot, msgType: `mavlink/${name}` });

describe('mavlink builders (round trip through the wire)', () => {
  test('commandLong puts every param on the wire (node-mavlink keeps them in _paramN)', () => {
    const wire = plainOf(commandLong({ targetSystem: 2, command: 176, params: [1, 4, 3, 4.5, 5, 6, 7.25] }), 255, 190);
    assert.deepEqual([wire.param1, wire.param2, wire.param3, wire.param4, wire.param5, wire.param6, wire.param7], [1, 4, 3, 4.5, 5, 6, 7.25]);
    assert.equal(wire.target_system, 2);
    assert.equal(wire.target_component, 1);
    assert.equal(wire.command, 176);
  });

  test('commandLong refuses broadcast (0), the GCS (255) and more than 7 params', () => {
    assert.throws(() => commandLong({ targetSystem: 0, command: 400 }), /vehicle sysid/);
    assert.throws(() => commandLong({ targetSystem: 255, command: 400 }), /vehicle sysid/);
    assert.throws(() => commandLong({ targetSystem: 1, command: 400, params: [1, 2, 3, 4, 5, 6, 7, 8] }), /7 params/);
  });

  test('setMessageInterval: message id in param1, period in µs in param2, 0 restores the default', () => {
    const at4 = plainOf(setMessageInterval(1, common.GlobalPositionInt, 4), 255, 190);
    assert.equal(at4.command, 511);
    assert.equal(at4.param1, 33);
    assert.equal(at4.param2, 250000);
    assert.equal(plainOf(setMessageInterval(1, common.HomePosition, 0), 255, 190).param2, 0);
  });

  test('classOfMsgType resolves mavlink/<NAME>, and nothing else', () => {
    assert.equal(classOfMsgType('mavlink/HOME_POSITION'), common.HomePosition);
    assert.equal(classOfMsgType('mavlink/NOPE'), undefined);
    assert.equal(classOfMsgType('sensor_msgs/NavSatFix'), undefined);
  });
});

describe('mavlink decoders', () => {
  const ARMED = 29 | 128;
  const px4Mode = (main, sub = 0) => ((sub << 24) | (main << 16)) >>> 0;

  test('PX4 heartbeat: disarmed in Hold, the code seen in SITL', () => {
    const p = decodeAs('vehicle_status', 'HEARTBEAT', heartbeat({ customMode: 50593792 }));
    assert.equal(p.armState, ARM_STATE.DISARMED);
    assert.equal(p.flightMode, FLIGHT_MODE.HOLD);
    assert.equal(p.flightModeRaw, 50593792);
    assert.equal(p.failsafe, false);
  });

  test('PX4 heartbeat: armed in Mission, Takeoff, precision land (sub 10), offboard', () => {
    const mode = (customMode) => decodeAs('vehicle_status', 'HEARTBEAT', heartbeat({ baseMode: ARMED, customMode }));
    assert.equal(mode(px4Mode(4, 4)).armState, ARM_STATE.ARMED);
    assert.equal(mode(px4Mode(4, 4)).flightMode, FLIGHT_MODE.MISSION);
    assert.equal(mode(px4Mode(4, 2)).flightMode, FLIGHT_MODE.TAKEOFF);
    assert.equal(mode(px4Mode(4, 10)).flightMode, FLIGHT_MODE.LAND);
    assert.equal(mode(px4Mode(4, 9)).flightMode, FLIGHT_MODE.AUTO_OTHER);
    assert.equal(mode(px4Mode(6)).flightMode, FLIGHT_MODE.OFFBOARD);
  });

  test('ArduCopter heartbeat: the mode number, read only for a multirotor', () => {
    const ardu = (customMode, type = minimal.MavType.QUADROTOR) =>
      decodeAs('vehicle_status', 'HEARTBEAT', heartbeat({ autopilot: minimal.MavAutopilot.ARDUPILOTMEGA, type, baseMode: 81, customMode }));
    assert.equal(ardu(4).flightMode, FLIGHT_MODE.OFFBOARD); // GUIDED
    assert.equal(ardu(5).flightMode, FLIGHT_MODE.HOLD); // LOITER
    assert.equal(ardu(6).flightMode, FLIGHT_MODE.RTL);
    assert.equal(ardu(9).flightMode, FLIGHT_MODE.LAND);
    assert.equal(ardu(3).flightMode, FLIGHT_MODE.MISSION); // AUTO
    assert.equal(ardu(4).armState, ARM_STATE.DISARMED);
    // Plane numbers its modes differently: 5 there is FBWA, not LOITER.
    assert.equal(ardu(5, minimal.MavType.FIXED_WING).flightMode, FLIGHT_MODE.OTHER);
  });

  test('a critical system status is a failsafe', () => {
    assert.equal(decodeAs('vehicle_status', 'HEARTBEAT', heartbeat({ systemStatus: minimal.MavState.CRITICAL })).failsafe, true);
  });

  test('GLOBAL_POSITION_INT: degrees, AMSL metres, heading and speed', () => {
    const p = decodeAs('position_global', 'GLOBAL_POSITION_INT', globalPosition({ lat: 43.4726046, lon: -8.1788475, altMm: 110250, hdg: 9050, vx: 300, vy: 400 }));
    assert.ok(Math.abs(p.latitude - 43.4726046) < 1e-7);
    assert.ok(Math.abs(p.longitude + 8.1788475) < 1e-7);
    assert.equal(p.altitude, 110.3);
    assert.equal(p.course, 90.5);
    assert.equal(p.speed, 5);
    assert.ok(p.deviceTime, 'keeps the device online (DeviceHealthMonitor)');
  });

  test('GLOBAL_POSITION_INT: unknown heading leaves the course alone; no fix (0/0) says nothing', () => {
    assert.equal('course' in decodeAs('position_global', 'GLOBAL_POSITION_INT', globalPosition({ hdg: 65535 })), false);
    assert.equal(decodeAs('position_global', 'GLOBAL_POSITION_INT', globalPosition({ lat: 0, lon: 0 })), null);
  });

  test('EXTENDED_SYS_STATE: MAV_LANDED_STATE is the flight phase', () => {
    const m = new common.ExtendedSysState();
    Object.assign(m, { vtolState: 0, landedState: common.MavLandedState.IN_AIR });
    const p = decodeAs('flight_status', 'EXTENDED_SYS_STATE', m);
    assert.equal(p.flightPhase, FLIGHT_PHASE.IN_AIR);
    assert.equal(p.flightPhaseRaw, 2);
  });

  test('BATTERY_STATUS: the first pack in percent; unknown or a second pack say nothing', () => {
    const battery = (id, remaining) => {
      const m = new common.BatteryStatus();
      Object.assign(m, { id, batteryFunction: 0, type: 0, temperature: 0, voltages: Array(10).fill(65535), currentBattery: -1, currentConsumed: -1, energyConsumed: -1, batteryRemaining: remaining });
      return decodeAs('battery', 'BATTERY_STATUS', m);
    };
    assert.equal(battery(0, 76).batteryLevel, 76);
    assert.equal(battery(0, -1), null);
    assert.equal(battery(1, 50), null);
  });

  test('COMMAND_ACK: MAV_RESULT in the GCS vocabulary', () => {
    const m = new common.CommandAck();
    Object.assign(m, { command: 400, result: 1, progress: 0, resultParam2: 0, targetSystem: 255, targetComponent: 190 });
    const p = decodeAs('vehicle_command_ack', 'COMMAND_ACK', m);
    assert.equal(p.resultCmdAck, COMMAND_RESULT.REJECTED);
    assert.equal(p.commandAckRaw, 400);
  });

  test('HOME_POSITION: the home the vehicle itself reports', () => {
    const m = new common.HomePosition();
    Object.assign(m, { latitude: 434726046, longitude: -81788475, altitude: 100208, x: 0, y: 0, z: 0, q: [1, 0, 0, 0], approachX: 0, approachY: 0, approachZ: 0 });
    assert.deepEqual(decodeAs('home_position', 'HOME_POSITION', m).home, [43.4726046, -8.1788475, 100.2]);
  });

  test('GIMBAL_DEVICE_ATTITUDE_STATUS: level and yaw-locked north is all zeros', () => {
    const m = new common.GimbalDeviceAttitudeStatus();
    Object.assign(m, { targetSystem: 0, targetComponent: 0, timeBootMs: 1, flags: 16, q: [1, 0, 0, 0], angularVelocityX: 0, angularVelocityY: 0, angularVelocityZ: 0, failureFlags: 0, deltaYaw: NaN, deltaYawVelocity: NaN, gimbalDeviceId: 0 });
    const { gimbal } = decodeAs('gimbal', 'GIMBAL_DEVICE_ATTITUDE_STATUS', m);
    assert.deepEqual([gimbal.x, gimbal.y, gimbal.z].map((v) => Math.abs(Math.round(v))), [0, 0, 0]);
  });

  test('every stream of the real mavlink categories names a known MAVLink message', () => {
    for (const name of ['px4_mavlink', 'ardupilot_mavlink']) {
      const subs = categoryModel.getCategory(name)?.subscribers;
      assert.ok(subs, `${name} exists`);
      for (const [slot, cfg] of Object.entries(subs)) {
        assert.ok(classOfMsgType(cfg.messageType), `${name}.${slot}: ${cfg.messageType}`);
      }
    }
  });
});

describe('MavlinkServer: telemetry and streams', () => {
  const CAT = {
    subscribers: {
      vehicle_status: { messageType: 'mavlink/HEARTBEAT' },
      position_global: { messageType: 'mavlink/GLOBAL_POSITION_INT', rate_hz: 4 },
      home_position: { messageType: 'mavlink/HOME_POSITION', rate_hz: 0.5 },
    },
  };
  const intervalsAsked = (link) =>
    link.sent.filter(({ msg }) => msg.command === 511).map(({ msg }) => ({ id: msg._param1, us: msg._param2 }));

  test('a declared message becomes a position update', async () => {
    const { feed, updates } = setup({ category: CAT });
    feed(heartbeat({ customMode: 50593792 }), 1);
    feed(globalPosition(), 1);
    await tick();
    assert.ok(updates.some((u) => u.armState === ARM_STATE.DISARMED && u.flightMode === FLIGHT_MODE.HOLD));
    assert.ok(updates.some((u) => u.latitude === 43.47));
  });

  test("a component's heartbeat on the vehicle's sysid is not the vehicle's state", async () => {
    const { feed, updates } = setup({ category: CAT });
    feed(heartbeat(), 1);
    await tick();
    updates.length = 0;
    feed(heartbeat({ autopilot: minimal.MavAutopilot.INVALID, type: minimal.MavType.CAMERA }), 1, { compid: 100 });
    await tick();
    assert.equal(updates.length, 0);
  });

  test('coming online asks for every rate_hz stream, and only those', async () => {
    const { feed, link } = setup({ category: CAT });
    feed(heartbeat(), 1);
    await tick();
    await tick();
    assert.deepEqual(intervalsAsked(link), [
      { id: 33, us: 250000 },
      { id: 242, us: 2000000 },
    ]);
    assert.ok(link.sent.every(({ remote }) => remote.port === 18570), 'sent back by the route the vehicle came from');
  });

  test('a stream is asked again only while it does not arrive, and not before the retry window', async () => {
    const { server, feed, link, advance, now } = setup({ category: CAT });
    feed(heartbeat(), 1);
    await tick();
    await tick();
    link.sent.length = 0;
    await server.streamTick(now());
    assert.equal(intervalsAsked(link).length, 0, 'just asked');

    advance(6000);
    feed(heartbeat(), 1);
    feed(globalPosition(), 1); // position arrives, home does not
    await server.streamTick(now());
    assert.deepEqual(intervalsAsked(link).map((r) => r.id), [242]);
  });

  test('nothing is asked for an unknown sysid', async () => {
    const { feed, link } = setup({ category: CAT, devicesBySysid: {} });
    feed(heartbeat(), 9);
    await tick();
    await tick();
    assert.equal(intervalsAsked(link).length, 0);
  });
});

// ─── F3: commands ─────────────────────────────────────────────────────────────

const POS = { latitude: 43.4726046, longitude: -8.1788475, altitude: 100.2, course: 90 };
const build = (dialect, type, attributes = {}, position = POS) => [buildMavlinkCommand({ dialect, type, attributes, position })].flat();

describe('mavlinkCommand: the dialects differ where SITL showed they do', () => {
  test('Takeoff on PX4: AMSL altitude (current + height), where it is', () => {
    const [c] = build(MAVLINK_COMMAND.PX4, CommandType.TAKEOFF, { altitude: 10 });
    assert.equal(c.kind, 'long');
    assert.equal(c.command, MAV_CMD.NAV_TAKEOFF);
    assert.ok(Math.abs(c.params[6] - 110.2) < 1e-9);
    assert.ok(Number.isNaN(c.params[4]) && Number.isNaN(c.params[5]), 'lat/lon NaN: take off where it is');
    assert.throws(() => build(MAVLINK_COMMAND.PX4, CommandType.TAKEOFF, { altitude: 10 }, null), /current position/);
  });

  test('Takeoff on ArduCopter: GUIDED, then COMMAND_INT relative frame with the height itself', () => {
    const [mode, takeoff] = build(MAVLINK_COMMAND.ARDUPILOT, CommandType.TAKEOFF, { altitude: 10 }, null);
    assert.deepEqual([mode.command, mode.params], [MAV_CMD.DO_SET_MODE, [1, 4]]);
    assert.equal(takeoff.kind, 'int');
    assert.equal(takeoff.frame, 3, 'GLOBAL_RELATIVE_ALT: anything else is DENIED');
    assert.equal(takeoff.alt, 10);
  });

  // LOITER follows the pilot's throttle stick: with no RC it descended to the ground in SITL.
  test('Hold, Pause and Stop: AUTO.LOITER on PX4, BRAKE (never LOITER) on ArduCopter', () => {
    for (const type of [CommandType.HOLD, CommandType.PAUSE_MISSION, CommandType.STOP_MISSION]) {
      assert.deepEqual(build(MAVLINK_COMMAND.PX4, type)[0].params, [1, 4, 3]);
      assert.deepEqual(build(MAVLINK_COMMAND.ARDUPILOT, type)[0].params, [1, 17]);
    }
  });

  test('KillMotors is a forced disarm on both', () => {
    for (const dialect of Object.values(MAVLINK_COMMAND)) {
      assert.deepEqual(build(dialect, CommandType.KILL_MOTORS)[0].params, [0, 21196]);
    }
  });

  test('MoveRelative on PX4: one COMMAND_INT, AMSL, yaw in RADIANS', () => {
    const cmds = build(MAVLINK_COMMAND.PX4, CommandType.MOVE_RELATIVE, { dx: 10, dy: 0, dz: 5, dyaw: 90 });
    assert.equal(cmds.length, 1);
    const [c] = cmds;
    assert.equal(c.kind, 'int');
    assert.equal(c.frame, 0);
    assert.ok(c.lon > POS.longitude, 'course 90: forward is east');
    assert.ok(Math.abs(c.alt - 105.2) < 1e-9);
    assert.ok(Math.abs(c.params[3] - Math.PI) < 1e-9, '90 + 90 = 180° = π rad');
  });

  test('MoveRelative on ArduCopter: the turn is a CONDITION_YAW in degrees (DO_REPOSITION ignores yaw)', () => {
    const [reposition, turn] = build(MAVLINK_COMMAND.ARDUPILOT, CommandType.MOVE_RELATIVE, { dx: 10, dy: 0, dz: 0, dyaw: 90 });
    assert.ok(Number.isNaN(reposition.params[3]));
    assert.equal(turn.command, MAV_CMD.CONDITION_YAW);
    assert.ok(Math.abs(turn.params[0] - 180) < 1e-9);
    assert.equal(build(MAVLINK_COMMAND.ARDUPILOT, CommandType.MOVE_RELATIVE, { dx: 10, dy: 0, dz: 0, dyaw: 0 }).length, 1, 'no turn, no CONDITION_YAW');
  });

  test('Gimbal on PX4: take the gimbal manager, then pitch/yaw, to any component', () => {
    const [configure, move] = build(MAVLINK_COMMAND.PX4, CommandType.GIMBAL, { pitch: -45, yaw: 10 });
    assert.equal(configure.command, MAV_CMD.DO_GIMBAL_MANAGER_CONFIGURE);
    assert.deepEqual(configure.params.slice(0, 2), [255, 190]);
    assert.equal(move.command, MAV_CMD.DO_GIMBAL_MANAGER_PITCHYAW);
    assert.deepEqual(move.params.slice(0, 2), [-45, 10]);
    assert.ok(Number.isNaN(move.params[2]), 'rate NaN: angle only');
    assert.equal(move.targetComponent, 0);
  });

  test('a COMMAND_INT descriptor puts x/y/z on the wire (node-mavlink keeps them in _param5..7)', () => {
    const [c] = build(MAVLINK_COMMAND.PX4, CommandType.MOVE_RELATIVE, { dx: 0, dy: 0, dz: 0, dyaw: 0 });
    const wire = plainOf(toCommandMessage(c, 1), 255, 190);
    assert.equal(wire.x, Math.round(POS.latitude * 1e7));
    assert.equal(wire.y, Math.round(POS.longitude * 1e7));
    assert.ok(Math.abs(wire.z - POS.altitude) < 1e-4);
    assert.equal(wire.frame, 0);
    assert.equal(wire.command, MAV_CMD.DO_REPOSITION);
  });

  test('commandInt refuses a missing coordinate instead of sending 0', () => {
    assert.throws(() => commandInt({ targetSystem: 1, command: 192, frame: 0, lat: NaN, lon: 1, alt: 1 }), /finite/);
  });
});

describe('commandRegistry: the mavlink categories', () => {
  test('Takeoff resolves to publishers.vehicle_command and encodes the ArduCopter sequence', () => {
    const category = categoryModel.getCategory('ardupilot_mavlink');
    const endpoint = commandRegistry.resolveCommands(category).commands.get(CommandType.TAKEOFF);
    assert.deepEqual(endpoint, { block: 'publishers', key: 'vehicle_command', msgType: MAVLINK_COMMAND.ARDUPILOT });
    const contents = contentsOf(commandRegistry.encode(CommandType.TAKEOFF, endpoint, { attributes: { altitude: 10 }, position: POS }));
    assert.deepEqual(contents.map((c) => c.command), [MAV_CMD.DO_SET_MODE, MAV_CMD.NAV_TAKEOFF]);
  });

  test('ArduCopter offers no Gimbal (not verified), PX4 does', () => {
    const has = (name) => commandRegistry.resolveCommands(categoryModel.getCategory(name)).commands.has(CommandType.GIMBAL);
    assert.equal(has('px4_mavlink'), true);
    assert.equal(has('ardupilot_mavlink'), false);
  });
});

describe('MavlinkServer: commands', () => {
  const PX4_ENDPOINT = { block: 'publishers', key: 'vehicle_command', msgType: MAVLINK_COMMAND.PX4 };
  const ARM = { kind: 'long', command: MAV_CMD.COMPONENT_ARM_DISARM, params: [1] };
  const fast = { commandAttemptMs: 40, statustextGraceMs: 10, gcsLinkWaitPolls: 0 };

  // A PX4 vehicle that has heard the GCS long enough to take commands.
  function ready({ autopilot = minimal.MavAutopilot.PX4, serverOpts = fast } = {}) {
    const ctx = setup({ serverOpts });
    ctx.feed(heartbeat({ autopilot }), 1);
    ctx.server.heartbeatTick(ctx.now());
    ctx.advance(1000);
    ctx.link.sent.length = 0;
    const answer = (fn) => (ctx.link.onSend = (msg) => msg.command !== 511 && setImmediate(() => fn(msg)));
    const send = (message = ARM, endpoint = PX4_ENDPOINT) => ctx.server.sendCommand({ device: DEVICE, endpoint, message });
    return { ...ctx, answer, send };
  }
  const commandsSent = (link) => link.sent.filter(({ msg }) => msg.command !== undefined && msg.command !== 511);

  test('ACCEPTED is a success, and what went out is the COMMAND_LONG of the descriptor', async () => {
    const { send, answer, feed, link } = ready();
    answer((msg) => feed(commandAck(msg.command, 0), 1));
    const r = await send();
    assert.deepEqual(r, { state: 'success', msg: 'COMPONENT_ARM_DISARM: ACCEPTED' });
    const [{ msg, remote }] = commandsSent(link);
    assert.equal(msg._param1, 1);
    assert.equal(msg.targetSystem, 1);
    assert.equal(remote.port, 18570);
  });

  test('a refusal carries the reason the vehicle gave in STATUSTEXT', async () => {
    const { send, answer, feed } = ready();
    answer((msg) => {
      feed(statusText(3, 'Arming denied: GCS connection lost\t'), 1);
      feed(commandAck(msg.command, 1), 1); // TEMPORARILY_REJECTED
    });
    const r = await send();
    assert.equal(r.state, 'error');
    assert.match(r.msg, /TEMPORARILY_REJECTED — Arming denied: GCS connection lost$/);
  });

  // STATUSTEXT holds 50 chars: a longer one is chunked, and a chunk may end in a space.
  test('a chunked STATUSTEXT is glued back whole, the space at the cut included', async () => {
    const { send, answer, feed } = ready();
    const first = 'PreArm: Battery 1 below minimum arming voltage and'; // 50 chars, ends before a space…
    answer((msg) => {
      feed(statusText(2, first, { id: 7, chunkSeq: 0 }), 1);
      feed(statusText(2, ' below the failsafe level', { id: 7, chunkSeq: 1 }), 1);
      feed(commandAck(msg.command, 4), 1);
    });
    assert.equal(first.length, 50);
    assert.match((await send()).msg, /FAILED — PreArm: Battery 1 below minimum arming voltage and below the failsafe level$/);
  });

  test('an informational STATUSTEXT is not a reason', async () => {
    const { send, answer, feed } = ready();
    answer((msg) => {
      feed(statusText(6, 'Armed by external command'), 1);
      feed(commandAck(msg.command, 4), 1); // FAILED
    });
    assert.equal((await send()).msg, 'COMPONENT_ARM_DISARM: FAILED');
  });

  test('a lost ACK is retried with confirmation++, and the retry can succeed', async () => {
    const { send, answer, feed, link } = ready();
    let n = 0;
    answer((msg) => n++ === 1 && feed(commandAck(msg.command, 0), 1));
    assert.equal((await send()).state, 'success');
    assert.deepEqual(commandsSent(link).map(({ msg }) => msg.confirmation), [0, 1]);
  });

  test('no ACK at all: an error after 3 attempts, never a success', async () => {
    const { send, link } = ready();
    const r = await send();
    assert.equal(r.state, 'error');
    assert.match(r.msg, /no COMMAND_ACK .* after 3 attempts/);
    assert.equal(commandsSent(link).length, 3);
  });

  test('IN_PROGRESS keeps the call open until the final ACK', async () => {
    const { send, answer, feed } = ready();
    answer((msg) => {
      feed(commandAck(msg.command, 5, { progress: 50 }), 1);
      setTimeout(() => feed(commandAck(msg.command, 0), 1), 20);
    });
    assert.equal((await send()).state, 'success');
  });

  test('an ACK addressed to another GCS is not ours', async () => {
    const { send, answer, feed } = ready();
    answer((msg) => feed(commandAck(msg.command, 0, { targetSystem: 200 }), 1));
    assert.equal((await send()).state, 'error');
  });

  test('a PX4 command to a vehicle whose heartbeat says ArduPilot is refused, nothing sent', async () => {
    const { send, link } = ready({ autopilot: minimal.MavAutopilot.ARDUPILOTMEGA });
    const r = await send();
    assert.equal(r.state, 'error');
    assert.match(r.msg, /reports ARDUPILOTMEGA; fix the device category/);
    assert.equal(commandsSent(link).length, 0);
  });

  test('an offline vehicle, or one that has not heard the GCS yet, gets nothing', async () => {
    const offline = setup({ serverOpts: fast });
    assert.match((await offline.server.sendCommand({ device: DEVICE, endpoint: PX4_ENDPOINT, message: ARM })).msg, /offline/);

    const fresh = setup({ serverOpts: fast });
    fresh.feed(heartbeat(), 1); // online, but no GCS heartbeat sent yet
    const r = await fresh.server.sendCommand({ device: DEVICE, endpoint: PX4_ENDPOINT, message: ARM });
    assert.match(r.msg, /has not heard the GCS heartbeat/);
    assert.equal(commandsSent(fresh.link).length, 0);
  });
});

// ─── F4: missions ─────────────────────────────────────────────────────────────

const C = MAV_CMD_MISSION;
const WPS = [
  { type: 'takeoff', pos: [43.4726, -8.1788, 110] },
  { type: 'inspection', pos: [43.4728, -8.1788, 112], speed: 3, gimbal: -30, action: { stay: 5, photo: 1 } },
  { type: 'inspection', pos: [43.4728, -8.1785, 112], yaw: 90 },
];
const task = (uav_type, params = {}, wp = WPS) => ({ task_id: 'T1', device: 'px4_1', uav_type, params, wp });
const commandsOf = (m) => m.items.map((i) => i.command);

describe('missionEncodeMavlink', () => {
  test('PX4: takeoff, speeds, waypoint with hold, gimbal and photo, land; the seq of each waypoint', () => {
    const m = encodeMavlinkMission(task('px4_mavlink'), MAVLINK_MISSION.PX4);
    assert.deepEqual(commandsOf(m), [
      C.NAV_TAKEOFF,
      C.DO_CHANGE_SPEED, // idle_vel
      C.DO_CHANGE_SPEED, // wp 1 speed
      C.NAV_WAYPOINT,
      C.DO_GIMBAL_MANAGER_PITCHYAW,
      C.IMAGE_START_CAPTURE,
      C.NAV_WAYPOINT,
      C.NAV_LAND, // default mode_landing: AUTO_LANDING
    ]);
    assert.deepEqual(m.wpSeqs, [0, 3, 6], 'the extra items do not count as waypoints');
    assert.equal(m.finish, MISSION_FINISH.LAND);
    assert.equal(m.items[3].params[0], 5, 'stay → hold time');
    assert.ok(Number.isNaN(m.items[6].params[3]), 'yaw AUTO: the autopilot faces the next waypoint');
    assert.deepEqual([m.items[7].lat, m.items[7].lon], [43.4728, -8.1785], 'lands at the last waypoint');
  });

  test('ArduPilot: seq 0 is the home placeholder, so every waypoint seq moves by one', () => {
    const m = encodeMavlinkMission(task('ardupilot_mavlink'), MAVLINK_MISSION.ARDUPILOT);
    assert.equal(m.items[0].command, C.NAV_WAYPOINT);
    assert.equal(m.items[1].command, C.NAV_TAKEOFF);
    assert.deepEqual(m.wpSeqs, [1, 4, 7]);
  });

  // AP_Mission::sanity_check_params: NaN only in param4 of NAV_WAYPOINT / TAKEOFF / LAND (and a few
  // others); anything else rejects the WHOLE upload (seen in SITL: INVALID_PARAM2 on the gimbal item).
  test('ArduPilot: no NaN where AP_Mission refuses it', () => {
    const NAN_PARAM4_OK = new Set([C.NAV_WAYPOINT, C.NAV_TAKEOFF, C.NAV_LAND]);
    const m = encodeMavlinkMission(task('ardupilot_mavlink', { mode_yaw: 3 }), MAVLINK_MISSION.ARDUPILOT);
    for (const [seq, item] of m.items.entries()) {
      item.params.forEach((p, i) => {
        const allowed = i === 3 && NAN_PARAM4_OK.has(item.command);
        assert.ok(allowed || !Number.isNaN(p), `seq ${seq} (MAV_CMD ${item.command}) param${i + 1} is NaN`);
      });
    }
  });

  test('waypoint yaw (WAYPOINT_CUSTOM): PX4 in NAV_WAYPOINT param4, ArduPilot as a CONDITION_YAW after it', () => {
    const px4 = encodeMavlinkMission(task('px4_mavlink', { mode_yaw: 3 }), MAVLINK_MISSION.PX4);
    assert.equal(px4.items[px4.wpSeqs[2]].params[3], 90);
    const ardu = encodeMavlinkMission(task('ardupilot_mavlink', { mode_yaw: 3 }), MAVLINK_MISSION.ARDUPILOT);
    const wp2 = ardu.wpSeqs[2];
    assert.ok(Number.isNaN(ardu.items[wp2].params[3]));
    assert.deepEqual([ardu.items[wp2 + 1].command, ardu.items[wp2 + 1].params[0]], [C.CONDITION_YAW, 90]);
  });

  test('finish: home → a final RTL item, hover → nothing after the last waypoint', () => {
    const home = encodeMavlinkMission(task('px4_mavlink', { mode_landing: 1 }), MAVLINK_MISSION.PX4);
    assert.equal(home.items.at(-1).command, C.NAV_RETURN_TO_LAUNCH);
    assert.equal(home.finish, MISSION_FINISH.HOME);
    const hover = encodeMavlinkMission(task('px4_mavlink', { mode_landing: 0 }), MAVLINK_MISSION.PX4);
    assert.equal(hover.items.at(-1).command, C.NAV_WAYPOINT);
    assert.equal(hover.finish, MISSION_FINISH.HOVER);
  });

  test('an option the family does not fly is refused, not approximated', () => {
    assert.throws(() => encodeMavlinkMission(task('px4_mavlink', { mode_trace: 0 }), MAVLINK_MISSION.PX4), /mode_trace=0.*not supported/);
    assert.throws(() => encodeMavlinkMission(task('px4_mavlink', { mode_landing: 3 }), MAVLINK_MISSION.PX4), /mode_landing=3/);
  });

  test('a route without a takeoff waypoint still takes off, unmapped to any waypoint', () => {
    const m = encodeMavlinkMission(task('px4_mavlink', {}, WPS.slice(1)), MAVLINK_MISSION.PX4);
    assert.equal(m.items[0].command, C.NAV_TAKEOFF);
    assert.equal(m.items[m.wpSeqs[0]].command, C.NAV_WAYPOINT);
  });

  test('the category exposes only the options the family maps', () => {
    const ids = (param) => categoryModel.getAtributesParam({ type: 'px4_mavlink', param }).map((o) => o.id).sort();
    assert.deepEqual(ids('mode_yaw'), [0, 3]);
    assert.deepEqual(ids('mode_trace'), [2]);
    assert.deepEqual(ids('mode_landing'), [0, 1, 2]);
  });

  test('PX4 restarts at item 0: only item 0 is flagged current; ArduPilot sends none', () => {
    // Without a current item PX4 resumes at the seq the previous mission ended on.
    const px4 = encodeMavlinkMission(task('px4_mavlink'), MAVLINK_MISSION.PX4);
    const flags = px4.items.map((item, seq) => plainOf(missionItemInt(1, seq, item), 255, 190).current);
    assert.deepEqual(flags, flags.map((_, i) => (i === 0 ? 1 : 0)));
    const ardu = encodeMavlinkMission(task('ardupilot_mavlink'), MAVLINK_MISSION.ARDUPILOT);
    assert.ok(ardu.items.every((item, seq) => plainOf(missionItemInt(2, seq, item), 255, 190).current === 0));
  });

  test('MISSION_ITEM_INT on the wire: lat/lon as ints in a global frame, raw x/y otherwise', () => {
    const m = encodeMavlinkMission(task('px4_mavlink'), MAVLINK_MISSION.PX4);
    const wp = plainOf(missionItemInt(1, 3, m.items[3]), 255, 190);
    assert.deepEqual([wp.x, wp.y, wp.seq, wp.frame, wp.autocontinue], [434728000, -81788000, 3, 0, 1]);
    assert.ok(Math.abs(wp.z - 112) < 1e-4);
    const gimbal = plainOf(missionItemInt(1, 4, m.items[4]), 255, 190);
    assert.deepEqual([gimbal.param1, gimbal.x, gimbal.frame], [-30, 0, 2]);
  });
});

describe('MavlinkServer: mission upload and progress', () => {
  const ENDPOINT = { block: 'services', key: 'configure_mission', msgType: MAVLINK_MISSION.PX4 };
  const fast = { commandAttemptMs: 40, statustextGraceMs: 10, gcsLinkWaitPolls: 0, uploadStepMs: 20 };

  function vehicle({ answer = true, rejectAt = null } = {}) {
    const ctx = setup({ serverOpts: fast });
    ctx.feed(heartbeat(), 1);
    ctx.server.heartbeatTick(ctx.now());
    ctx.advance(1000);
    ctx.link.sent.length = 0;
    const mission = encodeMavlinkMission(task('px4_mavlink'), MAVLINK_MISSION.PX4);
    const count = mission.items.length;
    if (answer) {
      ctx.link.onSend = (msg) =>
        setImmediate(() => {
          if (msg instanceof common.MissionCount) return ctx.feed(missionRequestInt(0), 1);
          if (!(msg instanceof common.MissionItemInt)) return;
          if (msg.seq === rejectAt) {
            ctx.feed(statusText(3, 'Mission rejected: item not supported'), 1);
            return ctx.feed(missionAck(common.MavMissionResult.UNSUPPORTED), 1);
          }
          ctx.feed(msg.seq + 1 < count ? missionRequestInt(msg.seq + 1) : missionAck(0), 1);
        });
    }
    const upload = () => ctx.server.sendCommand({ device: DEVICE, endpoint: ENDPOINT, message: mission });
    return { ...ctx, mission, upload };
  }
  const itemsSent = (link) => link.sent.filter(({ msg }) => msg instanceof common.MissionItemInt).map(({ msg }) => msg.seq);

  test('COUNT, every item the vehicle asks for, ACK: success, and the vehicle keeps the waypoint map', async () => {
    const { upload, link, server, mission } = vehicle();
    const r = await upload();
    assert.equal(r.state, 'success');
    assert.deepEqual(itemsSent(link), mission.items.map((_, i) => i));
    assert.deepEqual(server.getVehicle(1).mission.wpSeqs, [0, 3, 6]);
  });

  test('a rejection carries MAV_MISSION_RESULT and the STATUSTEXT, and loads nothing', async () => {
    const { upload, server } = vehicle({ rejectAt: 4 });
    const r = await upload();
    assert.equal(r.state, 'error');
    assert.match(r.msg, /UNSUPPORTED — Mission rejected: item not supported/);
    assert.equal(server.getVehicle(1).mission, null);
  });

  test('no answer: the COUNT is resent, then an error (never a silent success)', async () => {
    const { upload, link } = vehicle({ answer: false });
    const r = await upload();
    assert.equal(r.state, 'error');
    assert.match(r.msg, /no MISSION_REQUEST after MISSION_COUNT/);
    assert.equal(link.sent.filter(({ msg }) => msg instanceof common.MissionCount).length, 6);
  });

  // Seen in SITL: T2 started with T1's wp_reached=3 still cached. With a longer T2 route the
  // tracking would have jumped it to waypoint 3 before it flew.
  test('loading a mission resets wp_reached, so the previous mission cannot advance the new task', async () => {
    const { upload, updates } = vehicle();
    await upload();
    assert.deepEqual(updates.at(-1), { deviceId: 7, uav_state: 'ok', mission_state: 'loaded', wp_reached: 0 });
  });

  test('MISSION_ITEM_REACHED becomes wp_reached (count of waypoints), extra items do not', async () => {
    const { upload, feed, updates } = vehicle();
    await upload();
    updates.length = 0;
    feed(missionItemReached(3), 1); // waypoint 1
    feed(missionItemReached(4), 1); // the gimbal item
    assert.deepEqual(updates.map((u) => u.wp_reached), [2]);
  });

  // Seen in PX4 SITL (wpSeqs [0, 2, 6], RTL at 7): PX4 reported 1, 5 and 7, never a waypoint's own
  // seq, because it runs the items without a position after a waypoint before it reports it.
  test('PX4 reports the end of the waypoint group: any seq in [wp, next wp) is that waypoint', async () => {
    const { upload, feed, updates, finished, server } = vehicle();
    await upload();
    assert.deepEqual(server.getVehicle(1).mission.wpSeqs, [0, 3, 6]);
    updates.length = 0;
    feed(missionItemReached(1), 1); // takeoff, then its speed item
    feed(missionItemReached(1), 1); // PX4 repeats it every second
    feed(missionItemReached(5), 1); // waypoint 1, then gimbal, photo, speed
    feed(heartbeat({ baseMode: 29 | 128 }), 1);
    feed(missionItemReached(7), 1); // last waypoint, then the RTL
    assert.deepEqual(updates.map((u) => u.wp_reached), [1, 2, 3]);
    feed(heartbeat({ baseMode: 29 }), 1);
    assert.equal(finished.length, 1);
  });

  test('land at the end: finished only once the vehicle is down and disarmed', async () => {
    const { upload, feed, finished } = vehicle();
    await upload();
    feed(heartbeat({ baseMode: 29 | 128 }), 1);
    feed(missionItemReached(6), 1); // last waypoint, the NAV_LAND follows
    feed(heartbeat({ baseMode: 29 | 128 }), 1);
    assert.equal(finished.length, 0, 'still armed: landing');
    feed(heartbeat({ baseMode: 29 }), 1);
    assert.deepEqual(finished, [{ name: 'px4_1', id: 7 }]);
    feed(heartbeat({ baseMode: 29 }), 1);
    assert.equal(finished.length, 1, 'once');
  });

  test('hover at the end: finished at the last waypoint', async () => {
    const ctx = vehicle();
    ctx.mission.finish = MISSION_FINISH.HOVER;
    await ctx.upload();
    ctx.feed(missionItemReached(6), 1);
    assert.equal(ctx.finished.length, 1);
  });

  test('a new upload supersedes the one in progress', async () => {
    const { upload } = vehicle({ answer: false });
    const first = upload();
    const second = upload();
    assert.match((await first).msg, /superseded/);
    assert.equal((await second).state, 'error');
  });
});

describe('mavlink: the vehicle type of the HEARTBEAT vs the device model', () => {
  const MAV_TYPE_VTOL_TILTROTOR = 21;
  const run = async (type) => {
    const { server, feed } = setup({ devicesBySysid: { 1: [{ ...DEVICE, deviceModel: 'x500' }] } });
    server.deviceModels = { platformOf: (m) => (m === 'x500' ? { vehicleType: VEHICLE_TYPE.MULTIROTOR } : null) };
    const warned = [];
    server._warnOnce = (key, msg) => warned.push(msg);
    feed(heartbeat({ type }), 1);
    for (let i = 0; i < 5; i++) await tick();
    return { server, warned };
  };

  test('MAV_TYPE maps onto the GCS vehicle types; what is not a vehicle says nothing', () => {
    assert.equal(vehicleTypeOfMavType(minimal.MavType.QUADROTOR), VEHICLE_TYPE.MULTIROTOR);
    assert.equal(vehicleTypeOfMavType(minimal.MavType.FIXED_WING), VEHICLE_TYPE.FIXED_WING);
    assert.equal(vehicleTypeOfMavType(MAV_TYPE_VTOL_TILTROTOR), VEHICLE_TYPE.VTOL);
    assert.equal(vehicleTypeOfMavType(minimal.MavType.GROUND_ROVER), VEHICLE_TYPE.UGV);
    assert.equal(vehicleTypeOfMavType(minimal.MavType.SURFACE_BOAT), VEHICLE_TYPE.USV);
    assert.equal(vehicleTypeOfMavType(minimal.MavType.GIMBAL), null);
  });

  test('a contradiction is reported, never refused: the vehicle still comes online', async () => {
    const { server, warned } = await run(MAV_TYPE_VTOL_TILTROTOR);
    assert.equal(server.presenceOf(1), 'online');
    assert.equal(warned.length, 1);
    assert.match(warned[0], /MAV_TYPE 21 \(vtol\) but its device model 'x500' is a multirotor/);
  });

  test('a HEARTBEAT that matches the model says nothing', async () => {
    assert.deepEqual((await run(minimal.MavType.QUADROTOR)).warned, []);
  });
});
