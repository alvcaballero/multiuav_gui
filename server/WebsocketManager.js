//https://github.com/lukas8219/nodejs-design-patterns/blob/4a1d3cd4333a290e9461880ff2b060add40a1b45/13-messaging-and-integration-patterns/utils/websocket-manager.mjs#L4
//https://www.npmjs.com/package/ws#sending-binary-data  find "ping"

import { WebSocket, WebSocketServer } from 'ws';
import { logHelpers } from './common/logger.js';
import { WS_PING_INTERVAL_MS } from './config/config.js';

function heartbeat() {
  this.isAlive = true;
}

export class WebsocketManager {
  constructor(server, path = '/api/socket') {
    this.ws = new WebSocketServer({ path: path, server: server });

    // Handler externo para mensajes entrantes (lo setea el router vía onMessage).
    // El transporte NO parsea ni conoce tipos de negocio: solo delega el crudo.
    this.messageHandler = null;

    this.ws.on('connection', (client) => this._onConnection(client));

    this.interval_ping = setInterval(this.ping.bind(this), WS_PING_INTERVAL_MS);

    this.ws.on('close', () => {
      this._clearIntervals();
    });
  }

  /**
   * Registra el handler de mensajes entrantes. Recibe `(client, rawMessage)`,
   * donde `client` es el wrapper WebsocketClient (para poder responder).
   */
  onMessage(handler) {
    this.messageHandler = handler;
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

  _onConnection(rawClient) {
    rawClient.isAlive = true;
    logHelpers.ws.connect(this.ws.clients.size);

    const client = new WebsocketClient(this, { client: rawClient });

    rawClient.on('error', (error) => {
      logHelpers.ws.error('clientId', error);
    });

    rawClient.on('pong', heartbeat);

    rawClient.on('close', () => {
      logHelpers.ws.disconnect('Client disconnected');
    });

    // Delega el mensaje crudo al router externo; el transporte no lo interpreta.
    client.onMessage((rawMessage) => {
      if (this.messageHandler) {
        this.messageHandler(client, rawMessage);
      }
    });

    if (this.onClientConnect) {
      this.onClientConnect(rawClient);
    }
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
