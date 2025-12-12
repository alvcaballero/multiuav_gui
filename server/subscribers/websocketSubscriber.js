import { eventBus, EVENTS } from '../common/eventBus.js';
import logger from '../common/logger.js';

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
   * Configura todas las suscripciones a eventos
   */
  setupSubscriptions() {
    // Eventos de misiones
    this.subscribe(EVENTS.MISSION_CREATED, this.onMissionCreated.bind(this));
    this.subscribe(EVENTS.MISSION_UPDATED, this.onMissionUpdated.bind(this));
    this.subscribe(EVENTS.MISSION_INIT, this.onMissionInit.bind(this));

    // Eventos del sistema
    this.subscribe(EVENTS.EVENT_CREATED, this.onEventCreated.bind(this));

    // Eventos de posiciones (opcional, para futuro)
    this.subscribe(EVENTS.POSITION_UPDATED, this.onPositionUpdated.bind(this));
    this.subscribe(EVENTS.CAMERA_UPDATED, this.onCameraUpdated.bind(this));

    // Eventos de dispositivos
    this.subscribe(EVENTS.DEVICE_UPDATED, this.onDeviceUpdated.bind(this));

    // Eventos del servidor
    this.subscribe(EVENTS.SERVER_UPDATED, this.onServerUpdated.bind(this));

    // Eventos de chat
    this.subscribe(EVENTS.CHAT_USER_MESSAGE, this.onUserMessage.bind(this));
    this.subscribe(EVENTS.CHAT_ASSISTANT_MESSAGE, this.onMessageFromAssistant.bind(this));

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
   * Maneja la creación de misiones
   */
  onMissionCreated(mission) {
    logger.debug('WebSocketSubscriber: mission created', {
      missionId: mission.id,
      name: mission.name,
    });

    this.wsController.sendMessage({
      mission: {
        ...mission,
        name: mission.name || 'unnamed_mission',
      },
    });
  }

  /**
   * Maneja la actualización de misiones
   */
  onMissionUpdated(mission) {
    logger.debug('WebSocketSubscriber: mission updated', {
      missionId: mission.id,
    });

    this.wsController.sendMessage({
      mission: mission,
    });
  }

  /**
   * Maneja la inicialización de misiones
   */
  onMissionInit(mission) {
    logger.debug('WebSocketSubscriber: mission init', {
      missionId: mission.id,
    });

    this.wsController.sendMessage({
      mission: {
        ...mission,
        name: mission.name || 'name',
      },
    });
  }

  /**
   * Maneja la creación de eventos
   */
  onEventCreated(event) {
    logger.debug('WebSocketSubscriber: event created', {
      eventType: event.type,
      deviceId: event.deviceId,
    });

    this.wsController.sendMessage({
      events: [event],
    });
  }

  /**
   * Maneja actualizaciones de posiciones
   */
  onPositionUpdated(positions) {
    // Solo enviar si hay datos
    if (positions && Object.keys(positions).length > 0) {
      this.wsController.sendMessage({
        positions: positions,
      });
    }
  }

  /**
   * Maneja actualizaciones de cámara
   */
  onCameraUpdated(camera) {
    // Solo enviar si hay datos
    if (camera && Object.keys(camera).length > 0) {
      this.wsController.sendMessage({
        camera: Object.values(camera),
      });
    }
  }

  /**
   * Maneja actualizaciones de dispositivos
   */
  onDeviceUpdated(devices) {
    this.wsController.sendMessage({
      devices: Object.values(devices),
    });
  }

  /**
   * Maneja actualizaciones del servidor
   */
  onServerUpdated(serverState) {
    this.wsController.sendMessage({
      server: serverState,
    });
  }
  /**
   * Maneja mensajes del usuario (trigger LLM processing)
   */
  async onUserMessage(data) {
    const { chatController } = await import('../controllers/chat.js');

    logger.debug('WebSocketSubscriber: chat user message', {
      chatId: data.chatId,
      messageLength: data.message.length,
    });

    try {
      await chatController.processMessage(data);
    } catch (error) {
      logger.error('WebSocketSubscriber: Error in chatController.processMessage', {
        chatId: data.chatId,
        error: error.message,
        stack: error.stack,
      });
    }
  }

  /**
   * Maneja mensajes del asistente
   */
  onMessageFromAssistant(data) {
    const { chatId, message } = data;

    logger.debug('WebSocketSubscriber: chat message from assistant', {
      chatId,
      messageType: message.type || 'text',
    });

    this.wsController.sendMessage({
      chat: {
        chatId: chatId,
        from: 'assistant',
        message: message,
        timestamp: new Date().toISOString(),
      },
    });
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
