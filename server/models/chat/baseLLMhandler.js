export class BaseLLMHandler {
  constructor(apiKey, model, systemPrompt = '') {
    this.apiKey = apiKey;
    this.model = model;
    this.client = null;
    this.systemPrompt = systemPrompt;
    if (new.target === BaseLLMHandler) {
      throw new TypeError('Cannot construct BaseLLMHandler instances directly. Use a concrete subclass.');
    }
  }
  /**
   * Inicializa el cliente del LLM
   * Debe ser implementado por cada proveedor
   */
  async initialize() {
    throw new Error('initialize() debe ser implementado por la clase derivada');
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
  async processMessage(message, tools = [], conversationHistory = [], options = {}) {
    throw new Error('processMessage() debe ser implementado por la clase derivada');
  }

  /**
   * Ejecuta una llamada a herramienta solicitada por el LLM
   * @param {Object} toolCall - Información de la llamada a herramienta
   * @param {Function} toolExecutor - Función para ejecutar la herramienta
   * @returns {Promise<Object>} Resultado de la ejecución
   */
  async handleToolCall(toolCall, toolExecutor) {
    throw new Error('handleToolCall() debe ser implementado por la clase derivada');
  }

  /**
   * Normaliza la respuesta del LLM a un formato común
   * @param {any} response - Respuesta del LLM
   * @returns {Object} Respuesta normalizada
   */
  normalizeResponse(response) {
    throw new Error('normalizeResponse() debe ser implementado por la clase derivada');
  }

  /**
   * Obtiene el nombre del proveedor
   * @returns {string}
   */
  getProviderName() {
    throw new Error('getProviderName() debe ser implementado por la clase derivada');
  }
}
