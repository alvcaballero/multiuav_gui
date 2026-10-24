import * as fb from 'fbmsglib';
import * as flatbuffers from 'flatbuffers';
import { WebSocket, WebSocketServer } from 'ws';
import { parse } from 'url';
import { devicesController } from '../../controllers/devices.js';
import { positionsController } from '../../controllers/positions.js';
import { eventsController } from '../../controllers/events.js';
import {
  decodeFbMsg,
  decodeServiceResponse,
  isServiceResponse,
  getNameFromTopic,
  slotOfTopic,
  decodeFbViaCategory,
} from './fbDecode.js';
import { categoryModel } from '../category.js';
import { deviceModelsModel } from '../deviceModels/deviceModels.js';
import { unpackFbMsg } from './fbUnpack.js';
import { rawTap } from '../inspect/rawTap.js';
import { telemetryWatchdog } from '../inspect/telemetryWatchdog.js';
import { encodeFbMsg } from './fbEncode.js';
import { logger } from '../../common/logger.js';

const SERVICE_TIMEOUT_MS = 5000;

function getRobofleetMetadata(buf) {
  const msg = fb.fb.MsgWithMetadata.getRootAsMsgWithMetadata(buf);
  return msg._Metadata();
}

function toByteBuffer(data) {
  if (data instanceof Buffer) return new flatbuffers.ByteBuffer(data);
  if (data instanceof ArrayBuffer) return new flatbuffers.ByteBuffer(new Uint8Array(data));
  return null;
}

function heartbeat() {
  this.isAlive = true;
}

export class FlatbufferServer {
  constructor(port) {
    this.clients = new Map();
    this._serviceTimeouts = {};
    this._legacyWarned = new Set(); // `${deviceId}:${topic}` already reported by _warnLegacyDecoder

    this.ws = new WebSocketServer({ port });
    logger.info(`FlatbufferServer listening on port ${port}`);

    this._setupConnectionHandler();

    this._pingInterval = setInterval(this._ping.bind(this), 30000);
    this.ws.on('close', () => clearInterval(this._pingInterval));
  }

  // ── Public API ────────────────────────────────────────────────────────────

  async sendCommand({ uav_id, type, attributes }) {
    const device = await devicesController.getDevice(uav_id);
    let msg;
    try {
      msg = await encodeFbMsg({ uav_id, type, attributes });
    } catch (err) {
      logger.warn(`FlatbufferServer: cannot pack "${type}" for device "${device.name}": ${err.message}`);
      return { state: 'error', msg: err.message };
    }

    if (msg === null) {
      logger.warn(`FlatbufferServer: no encoder for type "${type}" on device "${device.name}"`);
      return { state: 'error', msg: 'command not found' };
    }

    this._sendToClient(device.name, msg);

    this._serviceTimeouts[`${device.name}_${type}`] = setTimeout(() => {
      this._handleServiceResponse({
        uav_id,
        name: device.name,
        type,
        response: { state: 'error', msg: 'timeout: no device confirmation' },
      });
    }, SERVICE_TIMEOUT_MS);

    return { state: 'success', msg: 'command sent' };
  }

  disconnect() {
    clearInterval(this._pingInterval);
    this.ws.close();
  }

  // ── Private ───────────────────────────────────────────────────────────────

  _setupConnectionHandler() {
    this.ws.on('connection', (socket, request) => {
      const name = parse(request.url, true).query['name'] || null;
      logger.info(`FlatbufferServer: new device connected name="${name}"`);

      socket.isAlive = true;
      socket.on('pong', heartbeat);
      socket.on('error', (err) => logger.error('FlatbufferServer socket error', { err }));
      socket.on('close', () => logger.info(`FlatbufferServer: device "${name}" disconnected`));
      socket.on('message', (data) => this._onMessage(data, name));

      if (name !== null) {
        this.clients.set(name, socket);
      }
    });
  }

  async _onMessage(data, name) {
    const buf = toByteBuffer(data);
    if (buf === null) return;

    const metadata = getRobofleetMetadata(buf) ?? null;
    if (metadata === null || metadata?.topic === null) return;

    const deviceName = name ?? getNameFromTopic(metadata.topic());

    const device = await devicesController.getByName(deviceName);
    if (!device) {
      logger.warn(`FlatbufferServer: message from unknown device "${deviceName}"`);
      return;
    }

    if (isServiceResponse(metadata)) {
      const result = decodeServiceResponse(metadata, buf, device.id, device.name);
      if (result) this._handleServiceResponse(result);
      return;
    }

    const subscribers = categoryModel.getCategory(device.category)?.subscribers;
    const slot = slotOfTopic(subscribers, metadata.topic());
    // A declared slot of a capability the device lacks is dropped (an undeclared topic keeps the old
    // per-type fallback, see fbDecode.js).
    if (subscribers?.[slot] && !deviceModelsModel.allowsSlot(device, slot)) return;

    // The unpack is lazy: rawTap only runs it for a message a client is actually watching.
    rawTap.publish({
      deviceId: device.id,
      slot,
      transport: 'robofleet',
      raw: () => unpackFbMsg(metadata.type(), buf),
    });
    telemetryWatchdog.record(device.id, slot);

    // Preferred: the category's decoder, same as rosbridge/MQTT. `undefined` = this topic is not
    // declared by the category (a bridge not renamed yet): the per-type decoder below handles it.
    let patch;
    try {
      patch = decodeFbViaCategory({
        type: metadata.type(),
        topic: metadata.topic(),
        buf,
        deviceId: device.id,
        subscribers,
      });
    } catch (err) {
      logger.warn(`FlatbufferServer: category decode failed for "${metadata.topic()}": ${err.message}`);
    }
    if (patch !== undefined) {
      if (patch) positionsController.updatePosition(patch);
      return;
    }

    this._warnLegacyDecoder(device, metadata);
    const position = decodeFbMsg(metadata, buf, device.id, device.category, device.name);
    if (position?.deviceId !== null) {
      positionsController.updatePosition(position);
    } else {
      logger.warn(`FlatbufferServer: unhandled msg type="${position?.type}" topic="${position?.topic}"`);
    }
  }

  // `decodeFbMsg` is the transitional per-type decoder: it only runs for a topic the device's
  // category does not declare (TODO.md: delete it). Say it ONCE per device and topic, so the log
  // shows what still needs it, and goes quiet when nothing does.
  _warnLegacyDecoder(device, metadata) {
    const key = `${device.id}:${metadata.topic()}`;
    if (this._legacyWarned.has(key)) return;
    this._legacyWarned.add(key);
    logger.warn(
      `FlatbufferServer: "${device.name}" (${device.category}) topic="${metadata.topic()}" ` +
        `type="${metadata.type()}" is decoded by the legacy per-type decoder, not by its category. ` +
        `Declare the topic in the category's subscribers and publish that name.`
    );
  }

  _handleServiceResponse({ uav_id, name, type, response }) {
    const key = `${name}_${type}`;
    if (this._serviceTimeouts[key]) {
      clearTimeout(this._serviceTimeouts[key]);
      delete this._serviceTimeouts[key];
    }
    eventsController.addEvent({
      type: response.state,
      deviceId: uav_id,
      attributes: { action: type, message: response.msg },
    });
  }

  _sendToClient(name, message) {
    const socket = this.clients.get(name);
    if (socket?.readyState === WebSocket.OPEN) {
      socket.send(message);
    }
  }

  _ping() {
    this.ws.clients.forEach((socket) => {
      if (socket.isAlive === false) return socket.terminate();
      socket.isAlive = false;
      socket.ping();
    });
  }
}
