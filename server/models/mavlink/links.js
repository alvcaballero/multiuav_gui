import dgram from 'node:dgram';
import net from 'node:net';
import { EventEmitter } from 'node:events';
import { PassThrough } from 'node:stream';
import { MavLinkPacketSplitter, MavLinkPacketParser, MavLinkProtocolV2 } from 'node-mavlink';
import { GCS_SYSTEM_ID, GCS_COMPONENT_ID } from './mavlinkMessages.js';
import { logger } from '../../common/logger.js';

// One link per physical channel (a UDP port, a TCP connection; serial with the SiK radios later).
// A link only frames bytes: it emits every parsed packet as `packet(packet, remote)` and sends
// messages back. It knows nothing about vehicles or devices — MavlinkServer does.
//
// The MAVLink `seq` is per channel, so each link owns its own sequence and protocol instance.

const TCP_RECONNECT_MS = 2000;
// A UDP remote that sent nothing for this long has its parser dropped (an ArduPilot restart
// comes back from a new ephemeral port; the old pipeline would leak).
const UDP_REMOTE_IDLE_MS = 60000;

/**
 * `udp:<bind host>:<port>` (listen) | `tcp:<host>:<port>` (connect).
 * @returns {{ kind: 'udp'|'tcp', host: string, port: number, id: string }}
 */
export function parseLinkSpec(spec) {
  const text = String(spec).trim();
  const match = /^(udp|tcp|serial):(.+):(\d+)$/.exec(text);
  if (!match) throw new Error(`invalid MAVLink link "${text}": expected udp:<host>:<port> or tcp:<host>:<port>`);
  const [, kind, host, port] = match;
  if (kind === 'serial') throw new Error(`MAVLink serial links are not supported yet ("${text}")`);
  return { kind, host, port: Number(port), id: text };
}

export const parseLinkSpecs = (specs) =>
  String(specs)
    .split(',')
    .map((s) => s.trim())
    .filter(Boolean)
    .map(parseLinkSpec);

/** Splitter → Parser. The pipeline emits synchronously inside write(). */
function createPipeline(onPacket) {
  const input = new PassThrough();
  input.pipe(new MavLinkPacketSplitter()).pipe(new MavLinkPacketParser()).on('data', onPacket);
  return input;
}

class FramingLink extends EventEmitter {
  constructor(spec) {
    super();
    this.spec = spec;
    this.id = spec.id;
    this.kind = spec.kind;
    this.protocol = new MavLinkProtocolV2(GCS_SYSTEM_ID, GCS_COMPONENT_ID);
    this.seq = 0;
  }

  _frame(message) {
    const buffer = this.protocol.serialize(message, this.seq);
    this.seq = (this.seq + 1) & 0xff;
    return buffer;
  }
}

/**
 * Listens on a UDP port; vehicles send to it (PX4 SITL from 18570+i, ArduPilot from an ephemeral
 * port). One pipeline per remote address:port, so the remote travels in the closure: a remote kept
 * in a shared variable is assigned AFTER the packet is emitted (the pipeline is synchronous).
 */
export class UdpLink extends FramingLink {
  constructor(spec, { createSocket = dgram.createSocket } = {}) {
    super(spec);
    this._pipelines = new Map(); // `address:port` -> { input, lastAt }
    // No reuseAddr: with it a second GCS (QGC, another server) binds the same port without error
    // and the kernel splits the vehicles' datagrams between both. EADDRINUSE at start is better.
    this.socket = createSocket({ type: 'udp4' });
    this.socket.on('message', (buffer, rinfo) => this._onDatagram(buffer, rinfo));
    this.socket.on('error', (err) => this.emit('error', err));
    this._pruneTimer = setInterval(() => this._prune(), UDP_REMOTE_IDLE_MS).unref();
  }

  start() {
    return new Promise((resolve, reject) => {
      this.socket.once('error', reject);
      this.socket.bind(this.spec.port, this.spec.host, () => {
        this.socket.off('error', reject);
        this.port = this.socket.address().port; // the real one when the spec asked for port 0
        logger.info(`MAVLink link ${this.id} listening on ${this.spec.host}:${this.port}`);
        resolve();
      });
    });
  }

  _onDatagram(buffer, rinfo) {
    const key = `${rinfo.address}:${rinfo.port}`;
    let entry = this._pipelines.get(key);
    if (!entry) {
      const remote = { address: rinfo.address, port: rinfo.port };
      entry = { input: createPipeline((packet) => this.emit('packet', packet, remote)), lastAt: 0 };
      this._pipelines.set(key, entry);
    }
    entry.lastAt = Date.now();
    entry.input.write(buffer);
  }

  _prune(now = Date.now()) {
    for (const [key, entry] of this._pipelines) {
      if (now - entry.lastAt > UDP_REMOTE_IDLE_MS) {
        entry.input.destroy();
        this._pipelines.delete(key);
      }
    }
  }

  /** @param {{address: string, port: number}} remote where the vehicle's packets come from */
  sendMessage(message, remote) {
    if (!remote) return false;
    this.socket.send(this._frame(message), remote.port, remote.address);
    return true;
  }

  close() {
    clearInterval(this._pruneTimer);
    for (const { input } of this._pipelines.values()) input.destroy();
    this._pipelines.clear();
    this.socket.close();
  }
}

/** Connects to a TCP endpoint (ArduPilot SITL serves one on 5760) and reconnects while open. */
export class TcpLink extends FramingLink {
  constructor(spec, { connect = net.connect } = {}) {
    super(spec);
    this._connect = connect;
    this._closed = false;
    this.socket = null;
  }

  start() {
    this._open();
    return Promise.resolve();
  }

  _open() {
    if (this._closed) return;
    const socket = this._connect({ host: this.spec.host, port: this.spec.port });
    // A new connection starts a new byte stream: a fresh splitter, not one holding a half frame.
    const input = createPipeline((packet) => this.emit('packet', packet, null));
    socket.on('connect', () => {
      this.socket = socket;
      logger.info(`MAVLink link ${this.id} connected`);
    });
    socket.on('data', (chunk) => input.write(chunk));
    socket.on('error', (err) => logger.debug(`MAVLink link ${this.id}: ${err.message}`));
    socket.on('close', () => {
      input.destroy();
      if (this.socket === socket) {
        this.socket = null;
        logger.warn(`MAVLink link ${this.id} disconnected`);
      }
      if (!this._closed) this._retry = setTimeout(() => this._open(), TCP_RECONNECT_MS).unref();
    });
  }

  sendMessage(message) {
    if (!this.socket) return false;
    this.socket.write(this._frame(message));
    return true;
  }

  close() {
    this._closed = true;
    clearTimeout(this._retry);
    this.socket?.destroy();
  }
}

export function createLink(spec) {
  return spec.kind === 'tcp' ? new TcpLink(spec) : new UdpLink(spec);
}
