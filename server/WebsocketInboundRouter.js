import { eventBus, EVENTS } from './common/eventBus.js';
import { logger, logHelpers } from './common/logger.js';
import { chatController } from './controllers/chat.js';
import { rawTap } from './models/inspect/rawTap.js';

/**
 * Router de mensajes ENTRANTES del cliente WebSocket (EIP Message Router).
 *
 * Es el dueño del inbound: traduce cada mensaje WS a una acción de dominio, ya
 * sea despachando un comando contra un controller o emitiendo un evento. El
 * transporte (`WebsocketManager`) NO conoce tipos de negocio: solo delega el
 * mensaje crudo `(client, raw)` a `handle`. Cualquier notificación de vuelta al
 * cliente sale por el EventBus → WebSocketSubscriber (outbound), nunca desde acá.
 *
 * Registry declarativo `type → handler`. Agregar un comando entrante es una
 * línea acá, no un `if` más en el transporte.
 */
const INBOUND_MAP = {
  [EVENTS.CHAT_USER_MESSAGE]: async (payload) => {
    const data = {
      chatId: payload.chatId,
      message: payload.message,
      timestamp: payload.timestamp,
      metadata: payload.metadata || {},
    };

    logger.debug('WS inbound: chat user message', {
      chatId: data.chatId,
      messageLength: data.message.length,
    });

    // El router ejecuta el comando directo contra el controller (dueño del inbound).
    const result = await chatController.processMessage(data);

    // Si se creó un chat nuevo (chatId venía vacío), la notificación de salida
    // sale por el EventBus → subscriber, no tocando el socket desde acá.
    if (!data.chatId && result.chatId) {
      eventBus.emitSafe(EVENTS.CHAT_CREATED, {
        chatId: result.chatId,
        timestamp: new Date().toISOString(),
      });
    }
  },

  // The ONLY way a parked flight tool gets authorised. A chat message never
  // approves anything, no matter what it says.
  [EVENTS.CHAT_TOOL_APPROVAL_RESPONSE]: async (payload) => {
    logger.info('WS inbound: tool approval response', {
      chatId: payload.chatId,
      count: payload.responses?.length ?? 0,
    });

    await chatController.respondToApproval({
      chatId: payload.chatId,
      responses: payload.responses,
      responderPrincipalId: payload.responderPrincipalId ?? null,
    });
  },

  // Inspector de mensajes raw: la suscripción es del cliente (`client.id`) y se
  // borra sola al cerrar el socket (server.js → onClientClose). Los mensajes salen
  // por RAW_MESSAGE → subscriber, dirigidos solo a este cliente, y la respuesta al
  // subscribe (aceptado o por qué no) por RAW_STATUS, igual que cualquier salida.
  [EVENTS.RAW_SUBSCRIBE]: async (payload, client) => {
    const deviceId = payload?.deviceId;
    const slot = payload?.slot;
    const result = rawTap.subscribe(client.id, deviceId, slot);
    if (!result.ok) logger.warn('WS inbound: raw subscribe rejected', { clientId: client.id, payload, ...result });
    eventBus.emitSafe(EVENTS.RAW_STATUS, { clientId: client.id, deviceId, slot, ...result });
    if (result.ok) rawTap.replayLast(client.id, deviceId, slot);
  },

  [EVENTS.RAW_UNSUBSCRIBE]: async (payload, client) => {
    rawTap.unsubscribe(client.id, payload?.deviceId, payload?.slot);
  },
};

export class WebsocketInboundRouter {
  constructor(map = INBOUND_MAP) {
    this.map = map;
  }

  /**
   * Parsea el mensaje crudo y lo despacha al handler registrado según su `type`.
   * @param {WebsocketClient} client - Cliente que originó el mensaje (para responder)
   * @param {Buffer|string} rawMessage - Mensaje crudo tal como llegó del socket
   */
  async handle(client, rawMessage) {
    let message;
    try {
      const messageStr = rawMessage.toString();
      logHelpers.ws.message('client', messageStr);
      message = JSON.parse(messageStr);
    } catch (error) {
      logHelpers.ws.error('Message parse error', error);
      return;
    }

    const handler = this.map[message.type];
    if (!handler) {
      logger.warn('WS inbound: no handler for message type', { type: message.type });
      return;
    }

    try {
      await handler(message.payload, client);
    } catch (error) {
      logHelpers.ws.error('Inbound handler error', error);
    }
  }
}
