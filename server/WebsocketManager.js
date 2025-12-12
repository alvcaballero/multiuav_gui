//https://github.com/lukas8219/nodejs-design-patterns/blob/4a1d3cd4333a290e9461880ff2b060add40a1b45/13-messaging-and-integration-patterns/utils/websocket-manager.mjs#L4
//https://www.npmjs.com/package/ws#sending-binary-data  find "ping"

import WebSocket, { WebSocketServer } from 'ws';
import { logHelpers } from './common/logger.js';

function heartbeat() {
  this.isAlive = true;
}

export class WebsocketManager {
  constructor(server, path = '/api/socket') {
    this.ws = new WebSocketServer({ path: path, server: server });

    this.onConnect(async (client) => {
      client.onMessage(async (rawMessage) => {
        try {
          const messageStr = rawMessage.toString();
          logHelpers.ws.message('client', messageStr);

          // Parse JSON message
          const message = JSON.parse(messageStr);

          // Route based on message type
          if (message.type === 'chat:user_message') {
            // Import EventBus dynamically to avoid circular dependencies
            const { eventBus, EVENTS } = await import('./common/eventBus.js');

            // Emit to EventBus for processing
            eventBus.emitSafe(EVENTS.CHAT_USER_MESSAGE, {
              chatId: message.payload.chatId,
              message: message.payload.message,
              timestamp: message.payload.timestamp,
              metadata: message.payload.metadata || {},
            });
          }
        } catch (error) {
          logHelpers.ws.error('Message parse error', error);
        }
      });
    });

    this.interval_ping = setInterval(this.ping.bind(this), 30000);

    this.ws.on('close', () => {
      this._clearIntervals();
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

  disconnect() {
    this._clearIntervals();
    this.ws.close();
  }

  _applyToAllClients(cb) {
    this.ws.clients.forEach(cb);
  }

  _clearIntervals() {
    clearInterval(this.interval_ping);
  }

  onConnect(cb) {
    this.ws.on('connection', (client) => {
      client.isAlive = true;
      logHelpers.ws.connect(this.ws.clients.size);

      client.on('error', (error) => {
        logHelpers.ws.error('clientId', error);
      });

      client.on('pong', heartbeat);

      client.on('close', () => {
        logHelpers.ws.disconnect('Client disconnected');
      });

      if (this.onClientConnect) {
        this.onClientConnect(client);
      }
      if (cb) {
        cb(new WebsocketClient(this, { client }));
      }
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
