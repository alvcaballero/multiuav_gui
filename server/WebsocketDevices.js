//https://github.com/lukas8219/nodejs-design-patterns/blob/4a1d3cd4333a290e9461880ff2b060add40a1b45/13-messaging-and-integration-patterns/utils/websocket-manager.mjs#L4
//https://www.npmjs.com/package/ws#sending-binary-data  find "ping"
import * as fb from './dist/schema_main.cjs';
import * as flatbuffers from 'flatbuffers'; // do not remove; needed by generated code
import WebSocket, { WebSocketServer } from 'ws';
import { devicesController } from './controllers/devices.js';
import { eventsController } from './controllers/events.js';
import { positionsController } from './controllers/positions.js';
import { parse } from 'url';
import { decoder, getNameFromTopic } from './WebsocketDecode.js';
import { encode } from './WebsocketEncode.js';
import { getDatetime } from './common/utils.js';
import { FbEnable } from './config/config.js';

let servicesTimeout = {};

function getRobofleetMetadata(buf) {
  const msg = fb.fb.MsgWithMetadata.getRootAsMsgWithMetadata(buf);
  return msg._Metadata();
}

export function getByteBuffer(data) {
  if (data instanceof Buffer) {
    return new flatbuffers.ByteBuffer(data);
  }
  if (data instanceof ArrayBuffer) {
    return new flatbuffers.ByteBuffer(new Uint8Array(data));
  }
  return null;
}

function heartbeat() {
  this.isAlive = true;
}

export function ServiceResponse({ uav_id, name, type, response }) {
  console.log('response command to ' + name + ' type:' + type);
  eventsController.addEvent({
    type: response.state,
    eventTime: getDatetime(),
    deviceId: uav_id,
    attributes: { message: response.msg },
  });
  if (servicesTimeout[`${name}_${type}`]) {
    clearTimeout(servicesTimeout[`${name}_${type}`]);
  }
}

export async function sendCommandToClient({ uav_id, type, attributes }) {
  if (!FbEnable) return { state: 'error', msg: 'flatbuffer connection is disabled' };

  let nameClient = await devicesController.getDevice(uav_id);
  console.log('send command to ' + nameClient.name + ' type:' + type);
  let msg = await encode({ uav_id, type, attributes });
  if (msg === null) return { state: 'error', msg: 'command not found to websocket' };
  WebsocketDevices._instance.sentToClient(nameClient.name, msg);
  servicesTimeout[`${nameClient.name}_${type}`] = setTimeout(
    ServiceResponse.bind(null, {
      uav_id,
      name: nameClient.name,
      type,
      response: { state: 'error', msg: 'timeout no device confirmation' },
    }),
    5000
  );
  return { state: 'success', msg: 'command sent' };
}

export class WebsocketDevices {
  constructor(port) {
    this.clients = new Map();
    console.log('init websocket devices ' + WebsocketDevices._instance);
    if (WebsocketDevices._instance) {
      console.log('return last instance singleton');
      return WebsocketDevices._instance;
    }
    console.log('new instance create');
    WebsocketDevices._instance = this;

    this.ws = new WebSocket.Server({ port: port });

    this.onConnect(async (client) => {
      let init_msg = 'Wellcome new device';
      client.notify(init_msg);

      client.onMessage(async (message, name) => {
        const buf = getByteBuffer(message);
        if (buf === null) {
          console.log("received message that wasn't a flatbuffer. ignoring.");
          return;
        }
        const metadata = getRobofleetMetadata(buf) ?? null;

        if (metadata === null) {
          console.log('received not flatbuffer msg. ignoring.');
          return;
        }

        if (metadata?.topic === null) {
          console.log('received message without topic. ignoring.');
          return;
        }
        // get device
        let deviceName = name;
        if (name == null) {
          deviceName = getNameFromTopic(metadata.topic());
          console.log('name space of device:' + name);
        }
        console.log('received message device %s on topic: %s - %s', deviceName, metadata.topic(), metadata.type());
        let device = await devicesController.getByName(deviceName);
        if (!device) {
          console.log('device not found');
          return;
        }
        positionsController.updatePosition(decoder(metadata, buf, device.id, device.category, device.name));
      });
    });

    const interval = setInterval(this.ping.bind(this), 30000);

    this.ws.on('close', function close() {
      clearInterval(interval);
    });
  }

  ping() {
    this._applyToAllClients((client) => {
      if (client.isAlive === false) return client.terminate();
      client.isAlive = false;
      client.ping();
    });
  }

  broadcast(message) {
    this._applyToAllClients((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(message);
      }
    });
  }

  broadcastToRoom(room, message) {
    const clients = this.clientsToRoom.get(room) || [];
    clients.forEach((client) => {
      client.notify(message);
    });
  }

  sentToClient(nameClient, message) {
    const client = this.clients.get(nameClient);
    if (client) {
      client.notify(message);
    }
    //this.clients.forEach((client) => {
    //  if (client.name === nameClient) {
    //    client.notify(message);
    //  }
    //});
  }

  disconnect() {
    this.ws.close();
  }

  _applyToAllClients(cb) {
    this.ws.clients.forEach(cb);
  }

  onConnect(cb) {
    this.ws.on('connection', (client, request) => {
      console.log(request.url);
      const name = parse(request.url, true).query['name'] || null;
      console.log('name:' + name);

      client.isAlive = true;

      console.log('New device is connected');

      client.on('error', console.error);

      client.on('pong', heartbeat);

      client.on('close', () => {
        console.log('device Connection closed');
      });
      let myWebsocketClient = new WebsocketClient(this, { client, name });
      this.clients.set(name, myWebsocketClient);

      cb(myWebsocketClient);
    });
  }
}

export class WebsocketClient {
  constructor(wsManager, { client, name }) {
    this.client = client;
    this.wsManager = wsManager;
    this.name = name;
  }

  notify(message) {
    if (this.client.readyState === WebSocket.OPEN) {
      this.client.send(message);
    }
  }

  onMessage(cb) {
    this.client.on('message', (message) => {
      cb(message, this.name);
    });
  }
}
