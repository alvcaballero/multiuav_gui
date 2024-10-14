//https://github.com/lukas8219/nodejs-design-patterns/blob/4a1d3cd4333a290e9461880ff2b060add40a1b45/13-messaging-and-integration-patterns/utils/websocket-manager.mjs#L4
//https://www.npmjs.com/package/ws#sending-binary-data  find "ping"

import WebSocket, { WebSocketServer } from 'ws';
import { websocketController } from './controllers/websocket.js';

function heartbeat() {
  this.isAlive = true;
}

export class WebsocketManager {
  constructor(server, path = '/api/socket') {
    console.log('init websocket manager' + WebsocketManager._instance);
    if (WebsocketManager._instance) {
      console.log('return last instance singleton');
      return WebsocketManager._instance;
    }
    console.log('new instance create');
    WebsocketManager._instance = this;

    this.ws = new WebSocket.Server({ path: path, server: server });

    this.onConnect(async (client) => {
      client.onMessage(async (message) => {
        console.log('received: %s', message.toString());
      });

      let init_msg = await websocketController.init();
      client.notify(init_msg);
    });

    const interval_update = setInterval(this.updateclient.bind(this), 250);

    const interval_server = setInterval(this.updateserver.bind(this), 5000);

    const interval = setInterval(this.ping.bind(this), 30000);

    this.ws.on('close', function close() {
      clearInterval(interval);
      clearInterval(interval_update);
      clearInterval(interval_server);
    });
  }

  async updateclient() {
    let msg = await websocketController.update();
    this._applyToAllClients((client) => {
      client.send(msg);
    });
  }

  async updateserver() {
    let msg = await websocketController.updateserver();
    this._applyToAllClients((client) => {
      client.send(msg);
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

var wsManager = null;

export function createWebsocketManager(server, path) {
  wsManager = new WebsocketManager(server, path);
  return wsManager;
}

export default wsManager;
