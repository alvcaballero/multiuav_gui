import { eventBus, EVENTS } from '../common/eventBus.js';
import { logger } from '../common/logger.js';

/**
 * Mapa declarativo evento → payload de salida.
 *
 * Cada entrada es una transformación pura `(data) => payload` que el dispatcher
 * genérico (`forward`) envía por WebSocket. Agregar un broadcast nuevo es una
 * línea acá, no un método más.
 *
 * El subscriber es 100% outbound: solo mapea eventos → mensajes WS. Los comandos
 * entrantes los maneja el WebsocketInboundRouter, no este archivo.
 */
const OUTBOUND_MAP = {
  // Misiones
  [EVENTS.MISSION_PLAN_SHOWN]: (mission) => ({
    missionPlan: { ...mission, name: mission.name || 'unnamed_mission' },
  }),
  [EVENTS.MISSION_UPDATED]: (data) => ({ missionUpdated: data }),
  [EVENTS.ROUTE_UPDATED]: (data) => ({ routeUpdated: data }),

  // Telemetría batcheada: positionBroadcastBatcher agrupa los devices que
  // cambiaron desde el último flush y emite esto cada WS_POSITIONS_INTERVAL_MS
  // (ver positionBroadcastBatcher.js). El snapshot completo al conectar va
  // aparte, por WelcomeMessage (no pasa por acá).
  [EVENTS.POSITION_UPDATED]: (positions) => (positions && Object.keys(positions).length ? { positions } : null),
  [EVENTS.DEVICE_UPDATED]: (devices) => ({ devices: Object.values(devices) }),
  [EVENTS.SERVER_UPDATED]: (serverState) => ({ server: serverState }),

  // Sistema
  [EVENTS.EVENT_CREATED]: (event) => ({ events: [event] }),

  // Chat
  [EVENTS.CHAT_CREATED]: (data) => ({ chatCreated: data }),
  [EVENTS.CHAT_BUSY]: ({ chatId, busy }) => ({ chatBusy: { chatId, busy } }),
  [EVENTS.CHAT_ASSISTANT_MESSAGE]: ({ chatId, message }) => ({
    chat: {
      chatId,
      from: 'assistant',
      message,
      timestamp: new Date().toISOString(),
    },
  }),
};

/**
 * Subscriber que escucha eventos del EventBus y los envía a través de WebSocket
 *
 * Este subscriber desacopla la lógica de negocio (models) del transporte (WebSocket).
 * Cualquier componente puede emitir eventos sin conocer cómo se transportan.
 */
export class WebSocketSubscriber {
  constructor(wsController) {
    if (!wsController) {
      throw new Error('WebSocketSubscriber requires a websocketController instance');
    }

    this.wsController = wsController;
    this.listeners = [];

    logger.info('WebSocketSubscriber initializing');
    this.setupSubscriptions();
  }

  /**
   * Configura todas las suscripciones a eventos: cada entrada del `OUTBOUND_MAP`
   * se registra como un broadcast declarativo. Sin handlers imperativos.
   */
  setupSubscriptions() {
    for (const [eventName, transform] of Object.entries(OUTBOUND_MAP)) {
      this.subscribe(eventName, (data) => this.forward(eventName, transform, data));
    }

    logger.info('WebSocketSubscriber subscriptions ready', {
      subscriptions: this.listeners.length,
    });
  }

  /**
   * Helper para suscribirse a eventos y mantener track
   */
  subscribe(eventName, handler) {
    const wrappedHandler = eventBus.onSafe(eventName, handler);
    this.listeners.push({ eventName, handler: wrappedHandler });
  }

  /**
   * Dispatcher genérico: aplica la transformación del mapa y envía el payload.
   * Si la transformación devuelve algo vacío, no envía nada.
   */
  forward(eventName, transform, data) {
    logger.debug('WebSocketSubscriber → outbound', { event: eventName });

    const payload = transform(data);
    if (payload == null) return;

    this.wsController.sendMessage(payload);
  }

  /**
   * Limpia todas las suscripciones
   */
  cleanup() {
    logger.info('WebSocketSubscriber cleanup', {
      listeners: this.listeners.length,
    });

    this.listeners.forEach(({ eventName, handler }) => {
      eventBus.removeListener(eventName, handler);
    });

    this.listeners = [];
  }
}
