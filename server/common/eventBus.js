import { EventEmitter } from 'events';
import logger from './logger.js';

/**
 * Sistema centralizado de eventos para desacoplar la comunicación entre componentes.
 *
 * Eventos disponibles:
 * - 'mission:created' - Nueva misión creada
 * - 'mission:updated' - Misión actualizada
 * - 'mission:init' - Misión inicializada
 * - 'event:created' - Nuevo evento del sistema
 * - 'position:updated' - Posición actualizada
 * - 'device:updated' - Dispositivo actualizado
 * - 'server:updated' - Estado del servidor actualizado
 * - 'chat:message' - Nuevo mensaje de chat del usuario
 * - 'chat:assistant_message' - Nuevo mensaje de chat del asistente
 */
class EventBus extends EventEmitter {
  constructor() {
    super();
    // Incrementar límite de listeners (por defecto es 10)
    this.setMaxListeners(50);

    this.setupLogging();
  }

  setupLogging() {
    // Log de todos los eventos emitidos (útil para debugging)
    const originalEmit = this.emit.bind(this);
    this.emit = (eventName, ...args) => {
      logger.debug('EventBus emit', {
        eventName,
        hasData: args.length > 0,
        listeners: this.listenerCount(eventName)
      });
      return originalEmit(eventName, ...args);
    };
  }

  /**
   * Emite un evento de forma segura, capturando errores de los listeners
   * @param {string} eventName - Nombre del evento
   * @param {any} data - Datos del evento
   * @returns {boolean} - true si el evento fue emitido correctamente
   */
  emitSafe(eventName, data) {
    try {
      const hasListeners = this.listenerCount(eventName) > 0;

      if (!hasListeners) {
        logger.warn('EventBus: No listeners for event', { eventName });
      }

      this.emit(eventName, data);
      return true;
    } catch (error) {
      logger.error('EventBus emit error', {
        eventName,
        error: error.message,
        stack: error.stack
      });
      return false;
    }
  }

  /**
   * Registra un listener con manejo automático de errores
   * @param {string} eventName - Nombre del evento
   * @param {Function} listener - Función callback
   */
  onSafe(eventName, listener) {
    const wrappedListener = async (...args) => {
      try {
        await listener(...args);
      } catch (error) {
        logger.error('EventBus listener error', {
          eventName,
          error: error.message,
          stack: error.stack
        });
      }
    };

    this.on(eventName, wrappedListener);
    return wrappedListener; // Retornar para poder hacer removeListener después
  }

  /**
   * Obtiene estadísticas del EventBus
   */
  getStats() {
    const events = this.eventNames();
    const stats = {
      totalEvents: events.length,
      events: {}
    };

    events.forEach(eventName => {
      stats.events[eventName] = {
        listeners: this.listenerCount(eventName)
      };
    });

    return stats;
  }

  /**
   * Limpia todos los listeners (útil para testing y shutdown)
   */
  cleanup() {
    logger.info('EventBus cleanup', this.getStats());
    this.removeAllListeners();
  }
}

// Singleton exportado
export const eventBus = new EventBus();

// Constantes de eventos para evitar typos
export const EVENTS = Object.freeze({
  // Misiones
  MISSION_CREATED: 'mission:created',
  MISSION_UPDATED: 'mission:updated',
  MISSION_INIT: 'mission:init',
  MISSION_STATUS_CHANGED: 'mission:status:changed',

  // Eventos del sistema
  EVENT_CREATED: 'event:created',

  // Posiciones
  POSITION_UPDATED: 'position:updated',
  CAMERA_UPDATED: 'camera:updated',

  // Dispositivos
  DEVICE_UPDATED: 'device:updated',

  // Servidor
  SERVER_UPDATED: 'server:updated',
  // Chat
  CHAT_USER_MESSAGE: 'chat:message',
  CHAT_ASSISTANT_MESSAGE: 'chat:assistant_message',
  // Sistema
  SYSTEM_ERROR: 'system:error',
  SYSTEM_SHUTDOWN: 'system:shutdown',
});
