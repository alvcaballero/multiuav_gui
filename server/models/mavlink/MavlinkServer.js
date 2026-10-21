import { EventEmitter } from 'node:events';
import { devicesController } from '../../controllers/devices.js';
import { positionsController } from '../../controllers/positions.js';
import { categoryModel } from '../category.js';
import { decodeMsg } from '../msgDecode.js';
import { rawTap } from '../inspect/rawTap.js';
import { telemetryWatchdog } from '../inspect/telemetryWatchdog.js';
import { logger } from '../../common/logger.js';
import { createLink as defaultCreateLink, parseLinkSpecs } from './links.js';
import {
  GCS_SYSTEM_ID,
  classOf,
  classOfMsgType,
  decode,
  isAutopilotHeartbeat,
  autopilotName,
  msgTypeOf,
  toPlain,
  gcsHeartbeat,
  setMessageInterval,
  toCommandMessage,
  isArmed,
  missionCount,
  missionItemInt,
  missionResultName,
} from './mavlinkMessages.js';
import { missionController } from '../../controllers/mission.js';
import { DIALECT_AUTOPILOT } from '../commands/mavlinkCommand.js';
import { common } from 'node-mavlink';

const HEARTBEAT_MS = 1000;
// How often the streams a category asks for (`rate_hz`) are checked, and re-requested when one
// is not arriving: a SET_MESSAGE_INTERVAL lost on the link (UDP, SiK), or refused while the
// vehicle was booting, or an ArduPilot restarted (it forgets every interval) without missing
// enough heartbeats to go offline.
const STREAM_CHECK_MS = 2000;
// A stream is missing once it has been silent for 3 periods, and never before this.
const STREAM_MIN_SILENCE_MS = 3000;
// Do not re-request the same stream faster than this: the ACK and the first message need time.
const STREAM_RETRY_MS = 5000;
const MAV_CMD_SET_MESSAGE_INTERVAL = 511;
const MAV_RESULT_ACCEPTED = 0;
const MAV_RESULT_IN_PROGRESS = 5;

// Commands. PX4 answers TEMPORARILY_REJECTED until it has heard the GCS for about a second (F0).
const GCS_LINK_SETTLE_MS = 1000;
const GCS_LINK_WAIT_POLLS = 30; // × 100 ms: how long a command waits for that before failing
const COMMAND_ATTEMPTS = 3; // a lost COMMAND_LONG/ACK is resent (confirmation++), not reported at once
const COMMAND_ATTEMPT_MS = 1500;
const COMMAND_IN_PROGRESS_MAX_MS = 15000; // IN_PROGRESS keeps the call open, up to this
// The reason of a refusal travels in STATUSTEXT ("PreArm: …", "Arming denied: …"), not in the
// ACK, and can arrive just after it.
const STATUSTEXT_GRACE_MS = 300;
const STATUSTEXT_KEEP_MS = 10000;
const MAV_SEVERITY_WARNING = 4;

// Mission upload: MISSION_COUNT → (MISSION_REQUEST_INT seq → MISSION_ITEM_INT seq)* → MISSION_ACK.
// A step with no answer resends what the vehicle is waiting for (the COUNT, or the last item).
const UPLOAD_STEP_MS = 1500;
const UPLOAD_RETRIES = 5;
const MAV_MISSION_ACCEPTED = 0;
const MISSION_FINISH_HOVER = 'hover';

const resultName = (code) => common.MavResult[code] ?? `MAV_RESULT ${code}`;
const commandName = (code) => common.MavCmd[code] ?? `MAV_CMD ${code}`;
const autopilotOfDialect = (msgType) => DIALECT_AUTOPILOT[msgType];
// PX4 and ArduPilot heartbeat at 1 Hz: five missed ones is a lost vehicle.
const PRESENCE_TIMEOUT_MS = 5000;
const DEVICE_CACHE_TTL_MS = 30000;
// A miss is cached briefly: a device created while its vehicle is already online must show up
// in seconds, not after the 30 s of a hit.
const DEVICE_MISS_TTL_MS = 3000;
// A misconfiguration (unknown sysid, sysid shared by two devices) is logged once per this window,
// not on every packet (tens of Hz).
const WARN_EVERY_MS = 60000;

/**
 * Fourth device transport: MAVLink over one or more links. Vehicles are told apart by their
 * system id (header.sysid), never by the link they came from: several vehicles share a UDP
 * port, and a vehicle is matched to its device by `device.systemId`.
 *
 * Per vehicle it keeps the route back (link + remote) learned from the autopilot's heartbeat,
 * and its presence. Emits `online` / `offline` with the vehicle.
 *
 * A message the device's category declares (a subscriber whose messageType is
 * `mavlink/<MESSAGE_NAME>`) goes to the raw inspector, the telemetry watchdog and the same
 * msgDecode registry every transport uses. A subscriber with `rate_hz` is a stream the server
 * asks the vehicle for (SET_MESSAGE_INTERVAL): ArduPilot sends no telemetry until asked, and PX4
 * sends position at ~50 Hz unless told otherwise.
 *
 * `createLink` and `deps` are injectable so tests run without sockets nor a database.
 */
export class MavlinkServer extends EventEmitter {
  constructor(
    specs,
    {
      createLink = defaultCreateLink,
      heartbeatMs = HEARTBEAT_MS,
      presenceTimeoutMs = PRESENCE_TIMEOUT_MS,
      commandAttemptMs = COMMAND_ATTEMPT_MS,
      statustextGraceMs = STATUSTEXT_GRACE_MS,
      gcsLinkWaitPolls = GCS_LINK_WAIT_POLLS,
      uploadStepMs = UPLOAD_STEP_MS,
      now = Date.now,
      deps = {},
    } = {}
  ) {
    super();
    this.presenceTimeoutMs = presenceTimeoutMs;
    this.commandAttemptMs = commandAttemptMs;
    this.statustextGraceMs = statustextGraceMs;
    this.gcsLinkWaitPolls = gcsLinkWaitPolls;
    this.uploadStepMs = uploadStepMs;
    // controllers/mission.js reaches this module through commands → gateways: resolved when a
    // mission finishes, never at load (same TDZ as the ROS gateway).
    this._missions = deps.missions;
    this.now = now;
    this.devices = deps.devices ?? devicesController;
    this.positions = deps.positions ?? positionsController;
    this.categories = deps.categories ?? categoryModel;
    this.rawTap = deps.rawTap ?? rawTap;
    this.watchdog = deps.telemetryWatchdog ?? telemetryWatchdog;

    // sysid -> { sysid, link, remote, online, lastHeartbeat, autopilot, gcsHeartbeatSince,
    //            lastMsgAt: Map(name -> ms), streamRequestedAt: Map(name -> ms) }
    this.vehicles = new Map();
    this._devices = new Map(); // sysid -> { device|null, at }
    this._warned = new Map(); // key -> last time it was logged

    const linkSpecs = Array.isArray(specs) ? specs : parseLinkSpecs(specs);
    this.links = linkSpecs.map((spec) => {
      const link = createLink(spec);
      link.on('packet', (packet, remote) => this._onPacket(link, packet, remote));
      link.on('error', (err) => logger.error(`MAVLink link ${link.id}: ${err.message}`));
      return link;
    });
    this.ready = Promise.all(this.links.map((l) => l.start?.())).catch((err) => {
      logger.error(`MAVLink link failed to start: ${err.message}`);
    });

    this._heartbeatTimer = setInterval(() => this.heartbeatTick(), heartbeatMs).unref();
    this._presenceTimer = setInterval(() => this.presenceTick(), Math.min(1000, presenceTimeoutMs)).unref();
    this._streamTimer = setInterval(() => this._streamTickSafe(), STREAM_CHECK_MS).unref();
    // A vehicle that (re)appears asks for every stream at once, without waiting for the tick.
    this.on('online', (v) => this.streamTick(this.now(), { only: v.sysid }).catch((err) => logger.error(`MAVLink streams sysid ${v.sysid}: ${err.message}`)));
  }

  // ── Public API ────────────────────────────────────────────────────────────

  /**
   * Sends one MAV_CMD (a descriptor built by models/commands/mavlinkCommand.js) and resolves
   * with the vehicle's verdict: `success` on ACCEPTED, `error` otherwise with the canonical result
   * and the STATUSTEXT that explains it. ACCEPTED means the vehicle took the command, not that the
   * manoeuvre finished: that is telemetry's job (missionSignals), as on every other protocol.
   * @param {{ device: object, endpoint: {block, key, msgType}, message: object }} args
   */
  async sendCommand({ device, endpoint, message }) {
    const expected = autopilotOfDialect(endpoint?.msgType);
    if (expected == null) {
      return { state: 'warning', msg: `${endpoint?.key}: not implemented over MAVLink yet` };
    }
    const v = this.vehicles.get(Number(device.systemId));
    if (!v?.online) return { state: 'error', msg: `${device.name}: vehicle sysid ${device.systemId} is offline` };
    // The category decides the dialect (what an altitude means); the heartbeat says the truth.
    if (v.autopilot !== expected) {
      return {
        state: 'error',
        msg: `${device.name}: its category sends ${autopilotName(expected)} commands but sysid ${v.sysid} reports ${autopilotName(v.autopilot)}; fix the device category`,
      };
    }
    if (!(await this._waitGcsLink(v))) {
      return { state: 'error', msg: `${device.name}: the vehicle has not heard the GCS heartbeat yet` };
    }
    if (message?.kind === 'mission') return this._uploadMission(v, message, device);
    try {
      toCommandMessage(message, v.sysid); // reject a bad descriptor before anything is sent
    } catch (err) {
      return { state: 'error', msg: err.message };
    }
    return this._commandWithAck(v, message);
  }

  /** @returns {'online'|'offline'|'unknown'} */
  presenceOf(systemId) {
    const v = this.vehicles.get(Number(systemId));
    if (!v) return 'unknown';
    return v.online ? 'online' : 'offline';
  }

  getVehicle(systemId) {
    return this.vehicles.get(Number(systemId));
  }

  /** Sends the GCS heartbeat once per route: per remote on UDP, once per connection on TCP. */
  heartbeatTick(now = this.now()) {
    const hb = gcsHeartbeat();
    for (const link of this.links) {
      const routes = new Map();
      for (const v of this.vehicles.values()) {
        if (v.link === link && v.online) routes.set(v.remote ? `${v.remote.address}:${v.remote.port}` : '-', v);
      }
      if (link.kind === 'tcp') {
        if (!link.sendMessage(hb)) continue;
        for (const v of routes.values()) v.gcsHeartbeatSince ??= now;
        continue;
      }
      for (const v of routes.values()) {
        if (link.sendMessage(hb, v.remote)) v.gcsHeartbeatSince ??= now;
      }
    }
  }

  presenceTick(now = this.now()) {
    for (const v of this.vehicles.values()) {
      if (v.online && now - v.lastHeartbeat > this.presenceTimeoutMs) {
        v.online = false;
        v.gcsHeartbeatSince = null;
        logger.warn(`MAVLink: sysid ${v.sysid} (${autopilotName(v.autopilot)}) lost, no heartbeat for ${now - v.lastHeartbeat} ms`);
        this.emit('offline', v);
      }
    }
  }

  /**
   * Asks each online vehicle for the streams its category declares with `rate_hz` that are not
   * arriving: never seen, or silent for 3 periods. A stream asked for less than STREAM_RETRY_MS
   * ago is left alone.
   * @param {{only?: number}} [opts] restrict to one sysid (a vehicle that just came online)
   */
  async streamTick(now = this.now(), { only } = {}) {
    for (const v of this.vehicles.values()) {
      if (!v.online || (only != null && v.sysid !== only)) continue;
      const device = await this._deviceFor(v.sysid);
      if (!device) continue;
      const subscribers = this.categories.getCategory(device.category)?.subscribers ?? {};
      for (const cfg of Object.values(subscribers)) {
        const hz = Number(cfg?.rate_hz);
        const clazz = classOfMsgType(cfg?.messageType);
        if (!(hz > 0) || !clazz) continue;
        const name = clazz.MSG_NAME;
        const lastMsg = v.lastMsgAt.get(name);
        const silentFor = lastMsg == null ? Infinity : now - lastMsg;
        if (silentFor <= Math.max(3000 / hz, STREAM_MIN_SILENCE_MS)) continue;
        const asked = v.streamRequestedAt.get(name);
        if (asked != null && now - asked < STREAM_RETRY_MS) continue;
        v.streamRequestedAt.set(name, now);
        logger.debug(`MAVLink: asking sysid ${v.sysid} for ${name} at ${hz} Hz`);
        this._send(v, setMessageInterval(v.sysid, clazz, hz));
      }
    }
  }

  close() {
    clearInterval(this._heartbeatTimer);
    clearInterval(this._presenceTimer);
    clearInterval(this._streamTimer);
    for (const v of this.vehicles.values()) {
      for (const w of [...v.ackWaiters]) w.done(null);
      v.upload?.finish({ state: 'error', msg: 'server shutting down' });
    }
    for (const link of this.links) link.close();
  }

  // ── Missions ──────────────────────────────────────────────────────────────

  /**
   * Uploads a mission (descriptor from missionEncodeMavlink.js) and resolves with the vehicle's
   * MISSION_ACK. On success the vehicle keeps `mission`: the seq of each waypoint, to turn
   * MISSION_ITEM_REACHED into wp_reached and to tell when the mission is over.
   */
  _uploadMission(v, descriptor, device) {
    let items;
    try {
      items = descriptor.items.map((item, seq) => missionItemInt(v.sysid, seq, item));
    } catch (err) {
      return Promise.resolve({ state: 'error', msg: err.message });
    }
    v.upload?.finish({ state: 'error', msg: 'superseded by a new mission upload' });
    const startedAt = this.now();

    return new Promise((resolve) => {
      const up = { requested: -1, retries: 0, timer: null };
      const rearm = () => {
        clearTimeout(up.timer);
        up.timer = setTimeout(onTimeout, this.uploadStepMs);
      };
      const onTimeout = () => {
        if (++up.retries > UPLOAD_RETRIES) {
          const where = up.requested < 0 ? 'no MISSION_REQUEST after MISSION_COUNT' : `stuck after item ${up.requested}`;
          return up.finish({ state: 'error', msg: `mission upload to sysid ${v.sysid}: no answer (${where})` });
        }
        this._send(v, up.requested < 0 ? missionCount(v.sysid, items.length) : items[up.requested]);
        rearm();
      };
      up.onRequest = (seq) => {
        if (!(seq >= 0 && seq < items.length)) return;
        up.requested = seq;
        up.retries = 0;
        this._send(v, items[seq]);
        rearm();
      };
      up.onAck = async (type) => {
        if (type === MAV_MISSION_ACCEPTED) {
          v.mission = {
            deviceId: device.id,
            deviceName: device.name,
            wpSeqs: descriptor.wpSeqs,
            lastWpSeq: descriptor.wpSeqs.at(-1),
            finish: descriptor.finish,
            reachedLast: false,
          };
          logger.info(`MAVLink: mission of ${items.length} items (${descriptor.wpSeqs.length} waypoints) loaded on sysid ${v.sysid}`);
          // The previous mission's wp_reached is still cached: with a longer new route
          // missionSignals would jump the new task to it before it flew. 0 = nothing reached.
          this.positions.updatePosition({ deviceId: device.id, uav_state: 'ok', mission_state: 'loaded', wp_reached: 0 });
          return up.finish({ state: 'success', msg: `mission loaded: ${items.length} items` });
        }
        clearTimeout(up.timer);
        await sleep(this.statustextGraceMs);
        const why = this._statusTextsSince(v, startedAt);
        up.finish({ state: 'error', msg: `mission upload rejected: ${missionResultName(type)}${why ? ` — ${why}` : ''}` });
      };
      up.finish = (result) => {
        clearTimeout(up.timer);
        if (v.upload === up) v.upload = null;
        resolve(result);
      };
      v.upload = up;
      this._send(v, missionCount(v.sysid, items.length));
      rearm();
    });
  }

  _onMissionMessage(v, name, msg) {
    // Requests and ACKs addressed to another GCS on the same link are not about our upload.
    if (msg.targetSystem !== 0 && msg.targetSystem !== GCS_SYSTEM_ID) return;
    if (name === 'MISSION_ACK') v.upload?.onAck(msg.type);
    else v.upload?.onRequest(msg.seq);
  }

  _onItemReached(v, seq) {
    const m = v.mission;
    if (!m) return;
    const index = m.wpSeqs.indexOf(seq);
    // wp_reached counts reached waypoints (missionSignals jumps the task to it): index + 1.
    if (index >= 0) this.positions.updatePosition({ deviceId: m.deviceId, uav_state: 'ok', mission_state: 'running', wp_reached: index + 1 });
    if (seq === m.lastWpSeq && !m.reachedLast) {
      m.reachedLast = true;
      if (m.finish === MISSION_FINISH_HOVER) this._finishMission(v);
    }
  }

  // RTL / land after the last waypoint: the mission is over once the vehicle is down and disarmed.
  _checkMissionLanded(v, heartbeat) {
    if (v.mission?.reachedLast && !isArmed(heartbeat)) this._finishMission(v);
  }

  _finishMission(v) {
    const m = v.mission;
    v.mission = null;
    logger.info(`MAVLink: ${m.deviceName} (sysid ${v.sysid}) finished its mission`);
    Promise.resolve((this._missions ?? missionController).deviceFinishMission({ name: m.deviceName, id: m.deviceId })).catch((err) =>
      logger.error(`MAVLink: finishing the mission of ${m.deviceName}: ${err.message}`)
    );
  }

  // ── Commands ──────────────────────────────────────────────────────────────

  async _waitGcsLink(v) {
    for (let i = 0; i <= this.gcsLinkWaitPolls; i++) {
      if (!v.online) return false;
      if (v.gcsHeartbeatSince != null && this.now() - v.gcsHeartbeatSince >= GCS_LINK_SETTLE_MS) return true;
      await sleep(100);
    }
    return false;
  }

  async _commandWithAck(v, descriptor) {
    const label = commandName(descriptor.command);
    const sentAt = this.now();
    for (let attempt = 0; attempt < COMMAND_ATTEMPTS; attempt++) {
      const ack = await this._sendAndWaitAck(v, toCommandMessage(descriptor, v.sysid, { confirmation: attempt }), descriptor.command);
      if (!ack) {
        logger.debug(`MAVLink: no ACK for ${label} from sysid ${v.sysid} (attempt ${attempt + 1})`);
        continue;
      }
      if (ack.result === MAV_RESULT_ACCEPTED) return { state: 'success', msg: `${label}: ${resultName(ack.result)}` };
      await sleep(this.statustextGraceMs);
      const why = this._statusTextsSince(v, sentAt);
      return { state: 'error', msg: `${label}: ${resultName(ack.result)}${why ? ` — ${why}` : ''}` };
    }
    return { state: 'error', msg: `${label}: no COMMAND_ACK from sysid ${v.sysid} after ${COMMAND_ATTEMPTS} attempts` };
  }

  // Resolves with the first final ACK for `command`, or null on timeout. IN_PROGRESS keeps it open.
  _sendAndWaitAck(v, message, command) {
    return new Promise((resolve) => {
      const started = this.now();
      const w = {
        command,
        onAck: (ack) => {
          if (ack.result !== MAV_RESULT_IN_PROGRESS) return w.done(ack);
          if (this.now() - started > COMMAND_IN_PROGRESS_MAX_MS) return w.done(null);
          clearTimeout(w.timer);
          w.timer = setTimeout(() => w.done(null), this.commandAttemptMs);
        },
        done: (ack) => {
          clearTimeout(w.timer);
          const i = v.ackWaiters.indexOf(w);
          if (i >= 0) v.ackWaiters.splice(i, 1);
          resolve(ack);
        },
      };
      w.timer = setTimeout(() => w.done(null), this.commandAttemptMs);
      v.ackWaiters.push(w);
      this._send(v, message);
    });
  }

  _onCommandAck(v, ack) {
    // An ACK addressed to another GCS on the same link is not the answer to ours.
    if (ack.targetSystem !== 0 && ack.targetSystem !== GCS_SYSTEM_ID) return;
    const waiter = v.ackWaiters.find((w) => w.command === ack.command);
    waiter?.onAck(ack);
  }

  // `text` is char[50], NUL-padded. A longer text comes in chunks with the same non-zero `id`
  // (chunk_seq 0, 1, …): they are glued back as they are, untrimmed — a chunk may end in the space
  // between two words. Trimming (PX4 ends its texts with a tab) happens when reading.
  _onStatusText(v, msg) {
    const now = this.now();
    const part = String(msg.text ?? '').replace(/\0/g, '');
    const last = v.statusTexts.at(-1);
    if (msg.id && last?.id === msg.id && msg.chunkSeq === last.nextChunk) {
      last.text += part;
      last.nextChunk++;
    } else {
      v.statusTexts.push({ at: now, severity: msg.severity, text: part, id: msg.id, nextChunk: 1 });
    }
    while (v.statusTexts.length && now - v.statusTexts[0].at > STATUSTEXT_KEEP_MS) v.statusTexts.shift();
  }

  /** The warnings and errors the vehicle said since `t`, joined: why a command was refused. */
  _statusTextsSince(v, t) {
    return v.statusTexts
      .filter((s) => s.at >= t && s.severity <= MAV_SEVERITY_WARNING)
      .map((s) => s.text.trim())
      .filter((text, i, all) => text && all.indexOf(text) === i)
      .join('; ');
  }

  // ── Private ───────────────────────────────────────────────────────────────

  _onPacket(link, packet, remote) {
    const { sysid } = packet.header;
    // 0 is broadcast and 255 is a GCS (this one, or another station on the same network).
    if (sysid === 0 || sysid === GCS_SYSTEM_ID) return;
    const clazz = classOf(packet);
    if (!clazz) return; // a message outside the loaded dialects (PX4 sends #441)

    let data;
    if (clazz.MSG_NAME === 'HEARTBEAT') {
      data = decode(packet);
      // A component's heartbeat (camera, gimbal) is not the vehicle's state: only the autopilot's goes on.
      if (!isAutopilotHeartbeat(clazz, data)) return;
      this._onAutopilotHeartbeat(link, remote, sysid, data);
    }
    // Only a vehicle whose autopilot heartbeat was seen has a route: the rest is not ours yet.
    const v = this.vehicles.get(sysid);
    if (!v) return;
    v.lastMsgAt.set(clazz.MSG_NAME, this.now());
    if (clazz.MSG_NAME === 'COMMAND_ACK') {
      data ??= decode(packet);
      this._checkStreamAck(v, data);
      this._onCommandAck(v, data);
    }
    if (clazz.MSG_NAME === 'STATUSTEXT') this._onStatusText(v, (data ??= decode(packet)));
    if (MISSION_PROTOCOL.has(clazz.MSG_NAME)) this._onMissionMessage(v, clazz.MSG_NAME, (data ??= decode(packet)));
    if (clazz.MSG_NAME === 'MISSION_ITEM_REACHED') this._onItemReached(v, (data ??= decode(packet)).seq);
    if (clazz.MSG_NAME === 'HEARTBEAT') this._checkMissionLanded(v, data);

    this._ingest(sysid, clazz, packet, data).catch((err) => logger.error(`MAVLink ingest sysid ${sysid}: ${err.message}`));
  }

  _checkStreamAck(v, ack) {
    if (ack.command !== MAV_CMD_SET_MESSAGE_INTERVAL || ack.result === MAV_RESULT_ACCEPTED) return;
    // The ACK does not say which message was refused; the stream tick asks again for what is missing.
    this._warnOnce(`stream:${v.sysid}:${ack.result}`, `MAVLink: sysid ${v.sysid} refused a SET_MESSAGE_INTERVAL (MAV_RESULT ${ack.result})`);
  }

  _send(v, message) {
    return v.link.sendMessage(message, v.remote);
  }

  _streamTickSafe() {
    this.streamTick().catch((err) => logger.error(`MAVLink stream check: ${err.message}`));
  }

  _onAutopilotHeartbeat(link, remote, sysid, heartbeat) {
    const now = this.now();
    let v = this.vehicles.get(sysid);
    if (!v) {
      v = {
        sysid,
        link,
        remote,
        online: false,
        lastHeartbeat: 0,
        autopilot: heartbeat.autopilot,
        gcsHeartbeatSince: null,
        lastMsgAt: new Map(),
        streamRequestedAt: new Map(),
        ackWaiters: [],
        statusTexts: [],
        upload: null, // the mission upload in progress
        mission: null, // the mission loaded: { deviceId, deviceName, wpSeqs, lastWpSeq, finish, reachedLast }
      };
      this.vehicles.set(sysid, v);
    }

    const sameRoute = v.link === link && sameRemote(v.remote, remote);
    if (!sameRoute) {
      if (v.online) {
        // Two live routes for one sysid: two vehicles sharing it, or one radio seen twice. Keep the
        // route we had; a sysid conflict must be fixed on the vehicles, not guessed here.
        this._warnOnce(`route:${sysid}`, `MAVLink: sysid ${sysid} seen from ${routeName(link, remote)} while live on ${routeName(v.link, v.remote)}: duplicated MAV_SYS_ID?`);
        return;
      }
      v.link = link;
      v.remote = remote;
    }

    v.lastHeartbeat = now;
    v.autopilot = heartbeat.autopilot;
    if (!v.online) {
      v.online = true;
      logger.info(`MAVLink: sysid ${sysid} (${autopilotName(heartbeat.autopilot)}) online via ${routeName(link, remote)}`);
      this.emit('online', v);
    }
  }

  // The slot is the subscriber of the device's category whose messageType is
  // `mavlink/<MESSAGE_NAME>`: a message the category does not declare is dropped here. A declared
  // one is turned into its plain form once, and that same object goes to the raw inspector and to
  // the decoder, as rosbridge's JSON does on the ROS side.
  async _ingest(sysid, clazz, packet, decoded) {
    const device = await this._deviceFor(sysid);
    if (!device) return;
    const subscribers = this.categories.getCategory(device.category)?.subscribers;
    const msgType = msgTypeOf(clazz.MSG_NAME);
    const slot = slotOf(subscribers, msgType);
    if (!slot) return;
    const msg = toPlain(clazz, decoded ?? decode(packet));
    this.rawTap.publish({ deviceId: device.id, slot, transport: 'mavlink', raw: msg });
    this.watchdog.record(device.id, slot);
    const patch = decodeMsg({ msg, deviceId: device.id, type: slot, msgType, params: subscribers[slot] });
    if (patch) this.positions.updatePosition(patch);
  }

  // Telemetry arrives at tens of Hz per vehicle: cache the sysid -> device lookup (a miss too).
  // The PROMISE is cached, not its result: otherwise every packet that arrives while the first
  // query is in flight starts another one.
  _deviceFor(sysid) {
    const hit = this._devices.get(sysid);
    if (hit && this.now() - hit.at < (hit.miss ? DEVICE_MISS_TTL_MS : DEVICE_CACHE_TTL_MS)) return hit.device;
    const entry = { at: this.now(), miss: false };
    entry.device = this._lookupDevice(sysid).then(
      (device) => {
        entry.miss = device == null;
        return device;
      },
      (err) => {
        this._devices.delete(sysid); // a failed query is retried by the next packet, not cached
        throw err;
      }
    );
    this._devices.set(sysid, entry);
    return entry.device;
  }

  async _lookupDevice(sysid) {
    const found = (await this.devices.getBySystemId(sysid)) ?? [];
    let device = null;
    if (found.length === 1) device = found[0];
    else if (found.length > 1) {
      this._warnOnce(`dup:${sysid}`, `MAVLink: devices ${found.map((d) => d.name).join(', ')} share sysid ${sysid}; ignoring it until only one has it`);
    } else {
      this._warnOnce(`unknown:${sysid}`, `MAVLink: sysid ${sysid} is not the systemId of any mavlink device`);
    }
    return device;
  }

  _warnOnce(key, msg) {
    const last = this._warned.get(key);
    if (last != null && this.now() - last < WARN_EVERY_MS) return;
    this._warned.set(key, this.now());
    logger.warn(msg);
  }
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

const MISSION_PROTOCOL = new Set(['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK']);

const sameRemote = (a, b) => (a == null && b == null) || (a != null && b != null && a.address === b.address && a.port === b.port);

const routeName = (link, remote) => (remote ? `${link.id} from ${remote.address}:${remote.port}` : link.id);

/** The subscriber key whose messageType is `msgType`, or undefined. */
export function slotOf(subscribers, msgType) {
  return Object.entries(subscribers ?? {}).find(([, cfg]) => cfg?.messageType === msgType)?.[0];
}
