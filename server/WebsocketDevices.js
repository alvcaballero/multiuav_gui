//https://github.com/lukas8219/nodejs-design-patterns/blob/4a1d3cd4333a290e9461880ff2b060add40a1b45/13-messaging-and-integration-patterns/utils/websocket-manager.mjs#L4
//https://www.npmjs.com/package/ws#sending-binary-data  find "ping"

import WebSocket, { WebSocketServer } from 'ws';
import { websocketController } from './controllers/websocket.js';

function heartbeat() {
  this.isAlive = true;
}

export class WebsocketDevices {
  constructor(port) {
    console.log('init websocket manager' + WebsocketManager._instance);
    if (WebsocketManager._instance) {
      console.log('return last instance singleton');
      return WebsocketManager._instance;
    }
    console.log('new instance create');
    WebsocketManager._instance = this;

    this.ws = new WebSocket.Server({ port: port });

    this.onConnect(async (client) => {
      client.onMessage(async (message) => {
        console.log('received: %s', message.toString());
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
