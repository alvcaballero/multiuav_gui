export const FORCE_FINISH_MESSAGE =
  'Maximum tool iterations reached. You MUST provide your final response NOW using only the information gathered so far. Do NOT attempt to call any more tools.';

export class BaseLLMHandler {
  constructor(apiKey, model, systemPrompt = '') {
    this.apiKey = apiKey;
    this.model = model;
    this.client = null;
    this.systemPrompt = systemPrompt;
    this.initialized = false;
    if (new.target === BaseLLMHandler) {
      throw new TypeError('Cannot construct BaseLLMHandler instances directly. Use a concrete subclass.');
    }
  }
  /**
   * Inicializa el cliente del LLM.
   * Subclasses should override, call their init logic, and set this.initialized = true.
   * Base implementation just sets the flag (for providers that init in the constructor).
   */
  async initialize() {
    this.initialized = true;
  }

  /**
   * Procesa un mensaje y retorna la respuesta
   * @param {string} message - Mensaje del usuario (null para continuación de tool calls)
   * @param {Array} tools - Herramientas disponibles
   * @param {Array} conversationHistory - Historial de conversación
   * @param {Object} options - Opciones adicionales para el procesamiento
   * @param {string} options.previousResponseId - ID de respuesta anterior para encadenar (evita enviar historial completo)
   * @param {string} options.instructions - Instrucciones del sistema (necesarias al usar previousResponseId)
   * @returns {Promise<Object>} Respuesta del LLM con formato { output: Array, responseId: string, model: string, status: string }
   */
  async processMessage(message, _tools = [], _conversationHistory = [], _options = {}) {
    throw new Error('processMessage() debe ser implementado por la clase derivada');
  }

  /**
   * Ejecuta una llamada a herramienta solicitada por el LLM
   * @param {Object} toolCall - Información de la llamada a herramienta
   * @param {Function} toolExecutor - Función para ejecutar la herramienta
   * @returns {Promise<Object>} Resultado de la ejecución
   */
  async handleToolCall(_toolCall, _toolExecutor) {
    throw new Error('handleToolCall() debe ser implementado por la clase derivada');
  }

  /**
   * Normaliza la respuesta del LLM a un formato común
   * @param {any} response - Respuesta del LLM
   * @returns {Object} Respuesta normalizada
   */
  normalizeResponse(_response) {
    throw new Error('normalizeResponse() debe ser implementado por la clase derivada');
  }

  /**
   * Obtiene el nombre del proveedor
   * @returns {string}
   */
  getProviderName() {
    throw new Error('getProviderName() debe ser implementado por la clase derivada');
  }

  /**
   * Ensures a provider-specific session exists for the given chat.
   * Providers that support persistent sessions (e.g., OpenAI Conversations API)
   * should override this to create/retrieve a session ID.
   * @param {string} chatId - Internal chat identifier
   * @param {Object} persistence - Adapter with { getSessionId, setSessionId, clearSession }
   * @returns {Promise<string|null>} Session ID or null if provider doesn't use sessions
   */
  async ensureSession(_chatId, _persistence) {
    return null;
  }

  /**
   * Handles session-related errors after a failed processMessage call.
   * Providers should override this to recover from expired/invalid sessions.
   * @param {string} chatId - Internal chat identifier
   * @param {Error} error - The error from processMessage
   * @param {Object} persistence - Adapter with { getSessionId, setSessionId, clearSession }
   * @returns {Promise<boolean>} true if the session was recovered (caller should retry), false otherwise
   */
  async handleSessionError(_chatId, _error, _persistence) {
    return false;
  }

  /**
   * Resolves the provider-specific model config for an agent based on its capability tier.
   * Subclasses define CAPABILITY_MAP: { low, medium, high } → { model, reasoning, ... }
   * @param {{ capability: string }} agent - Agent definition from agents/index.js
   * @returns {Object} Config { model, reasoning, maxTokens, ... }
   */
  resolveModelConfig(agent) {
    const tier = agent?.capability ?? 'low';
    const map = this.constructor.CAPABILITY_MAP || {};
    return map[tier] || map['low'] || { model: this.model };
  }

  static CAPABILITY_MAP = {
    low: {},
    medium: {},
    high: {},
  };
}
