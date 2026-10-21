//https://github.com/lukas8219/nodejs-design-patterns/blob/4a1d3cd4333a290e9461880ff2b060add40a1b45/13-messaging-and-integration-patterns/utils/websocket-manager.mjs#L4
//https://www.npmjs.com/package/ws#sending-binary-data  find "ping"

import { randomUUID } from 'node:crypto';
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

    // Clientes efímeros: viven mientras dure la conexión. `id` → WebsocketClient;
    // se alta en `connection` y se baja en `close`.
    this.clients = new Map();

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

  /** Wrapper del cliente conectado con ese id, o undefined si ya se desconectó. */
  getClient(id) {
    return this.clients.get(id);
  }

  /** Unicast por id. Devuelve false si el cliente ya no existe o no está abierto. */
  sendTo(id, message) {
    const client = this.clients.get(id);
    if (!client || client.client.readyState !== WebSocket.OPEN) return false;
    client.notify(message);
    return true;
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

    const client = new WebsocketClient(this, { client: rawClient, id: randomUUID() });
    this.clients.set(client.id, client);
    logHelpers.ws.connect(client.id, { total: this.clients.size });

    rawClient.on('error', (error) => {
      logHelpers.ws.error(client.id, error);
    });

    rawClient.on('pong', heartbeat);

    rawClient.on('close', () => {
      this.clients.delete(client.id);
      logHelpers.ws.disconnect(client.id, { total: this.clients.size });
      if (this.onClientClose) {
        this.onClientClose(client.id);
      }
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
  constructor(wsManager, { client, id }) {
    this.id = id;
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
