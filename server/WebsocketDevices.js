//https://github.com/lukas8219/nodejs-design-patterns/blob/4a1d3cd4333a290e9461880ff2b060add40a1b45/13-messaging-and-integration-patterns/utils/websocket-manager.mjs#L4
//https://www.npmjs.com/package/ws#sending-binary-data  find "ping"
import * as fb from './dist/schema_generated.cjs';
import * as flatbuffers from 'flatbuffers'; // do not remove; needed by generated code
import WebSocket, { WebSocketServer } from 'ws';
import { websocketController } from './controllers/websocket.js';

function getRobofleetMetadata(buf) {
  const msg = fb.fb.MsgWithMetadata.getRootAsMsgWithMetadata(buf);
  return msg._metadata();
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

export class WebsocketDevices {
  constructor(port) {
    console.log('init websocket manager' + WebsocketDevices._instance);
    if (WebsocketDevices._instance) {
      console.log('return last instance singleton');
      return WebsocketDevices._instance;
    }
    console.log('new instance create');
    WebsocketDevices._instance = this;

    this.ws = new WebSocket.Server({ port: port });

    this.onConnect(async (client) => {
      client.onMessage(async (message) => {
        console.log('received: %s', message.toString());
        const buf = getByteBuffer(message);
        if (buf === null) {
          console.log("received message that wasn't a flatbuffer. ignoring.");
          return;
        }
        const topic = getRobofleetMetadata(buf)?.topic() ?? null;
        const metadata = getRobofleetMetadata(buf) ?? null;
        console.log('received message on topic: %s', topic);
        if ((metadata === null || metadata === void 0 ? void 0 : metadata.type()) === 'sensor_msgs/NavSatFix') {
          console.log('received navsatfix message');
          const msg = fb.fb.sensor_msgs.NavSatFix.getRootAsNavSatFix(buf);
          console.log('latitude: %f, longitude: %f', msg.latitude(), msg.longitude());
        }
      });
      let init_msg = 'hi from server';
      client.notify(init_msg);
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

  disconnect() {
    this.ws.close();
  }

  _applyToAllClients(cb) {
    this.ws.clients.forEach(cb);
  }

  onConnect(cb) {
    this.ws.on('connection', (client) => {
      client.isAlive = true;
      console.log('newclient');

      client.on('error', console.error);

      client.on('pong', heartbeat);

      client.on('close', () => {
        console.log('Connection closed');
      });

      cb(new WebsocketClient(this, { client }));
    });
  }
}

export class WebsocketClient {
  constructor(wsManager, { client }) {
    this.client = client;
    this.wsManager = wsManager;
  }

  notify(message) {
    if (this.client.readyState === WebSocket.OPEN) {
      this.client.send(message);
    }
  }

  onMessage(cb) {
    this.client.on('message', cb);
  }
}
