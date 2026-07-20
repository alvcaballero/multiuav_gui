import { eventBus, EVENTS } from './common/eventBus.js';
import { logger, logHelpers } from './common/logger.js';
import { chatController } from './controllers/chat.js';

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
