/** Marker prefix that tells the model a message is a system directive, not user input. */
export const SYSTEM_DIRECTIVE_MARKER = '[SYSTEM_DIRECTIVE]';

/**
 * Text sent when the tool loop hits its iteration cap.
 *
 * Delivered as a `user`-role message in EVERY provider, deliberately:
 *  - Anthropic rejects `role: 'system'` inside `messages` (system is top-level);
 *  - Gemini drops system-role history entries (they go via `systemInstruction`);
 * so `user` is the only role all four replay mid-conversation. The marker is what
 * carries the "this is the system talking" meaning that the role cannot.
 *
 * EPHEMERAL: built while assembling a request, never persisted. A stored copy
 * would keep ordering the model to wrap up on every later turn.
 */
export const FORCE_FINISH_MESSAGE =
  `${SYSTEM_DIRECTIVE_MARKER} Maximum tool iterations reached. You MUST provide your final response NOW ` +
  'using only the information gathered so far. Do NOT attempt to call any more tools. ' +
  'Summarize what was accomplished and present the results to the user.';

/**
 * The force-finish directive as a turn-input item, ready to be appended to a
 * `tool_output` turnInput's `items` array by the orchestrator. Each handler
 * picks it out of `items` (by `type`) and inserts it wherever its provider's
 * API requires — merged into the same content array for Anthropic, as a
 * separate message for the rest. See each handler's processMessage.
 *
 * @returns {{type: 'directive', role: string, content: string}}
 */
export function forceFinishItem() {
  return { type: 'directive', role: 'user', content: FORCE_FINISH_MESSAGE };
}

/**
 * Builds the canonical usage shape every handler must return.
 * `raw` is kept verbatim so a bug in normalization never destroys the original numbers.
 * @param {object} fields
 * @returns {{input:number, output:number, cached:number, reasoning:number, total:number, raw:object|null}}
 */
export function makeUsage({ input = 0, output = 0, cached = 0, reasoning = 0, total = null, raw = null } = {}) {
  const n = (v) => (Number.isFinite(Number(v)) ? Number(v) : 0);
  const inputTokens = n(input);
  const outputTokens = n(output);
  return {
    input: inputTokens,
    output: outputTokens,
    cached: n(cached),
    reasoning: n(reasoning),
    total: total === null || total === undefined ? inputTokens + outputTokens : n(total),
    raw,
  };
}

/** Marker prefix that tells the model a message is a system-delivered subagent result. */
export const SUBAGENT_RESULT_MARKER = '[SUBAGENT_RESULT]';

/**
 * Renders a canonical `subagent_result` payload as the plain text every provider
 * receives. Subagents answer asynchronously, long after the parent's tool pair
 * closed, so their result travels as an ordinary user-role message instead of a
 * function_call_output — no provider has a protocol slot for a late tool reply.
 *
 * The envelope (agent/tool) is built from fields WE control; everything the
 * subagent produced stays inside the fenced JSON, where a stray marker string
 * cannot impersonate a new envelope.
 *
 * @param {object} item - Canonical subagent_result payload
 * @returns {string} Text to hand to the provider
 */
export function renderSubagentResult(item) {
  const agent = item.agentName || 'unknown';
  const tool = item.name || 'unknown';
  const output = typeof item.output === 'string' ? item.output : JSON.stringify(item.output ?? {});
  return (
    `${SUBAGENT_RESULT_MARKER} agent=${agent} tool=${tool}\n` +
    'This is the result of a subagent you dispatched earlier, delivered by the ' +
    'system. It is NOT a message from the user. Treat it as tool output.\n' +
    '```json\n' +
    output +
    '\n```'
  );
}

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
   * Procesa un turno y retorna la respuesta.
   * @param {?{type: 'message', content: *}|{type: 'tool_output', items: Array}} turnInput - Contenido del
   *   turno, o null para continuar puramente desde el historial persistido (p.ej. subagent_result).
   *   Un item de `tool_output.items` con `type: 'directive'` es la instrucción de force-finish —
   *   ver `forceFinishItem()` — y cada handler decide dónde insertarla según su API.
   * @param {Array} tools - Herramientas disponibles
   * @param {Array} conversationHistory - Historial de conversación
   * @param {Object} options - Opciones adicionales para el procesamiento
   * @param {string} options.previousResponseId - ID de respuesta anterior para encadenar (evita enviar historial completo)
   * @param {string} options.instructions - Instrucciones del sistema (necesarias al usar previousResponseId)
   * @returns {Promise<Object>} Respuesta del LLM con formato { output: Array, responseId: string, model: string, status: string }
   */
  async processMessage(turnInput, _tools = [], _conversationHistory = [], _options = {}) {
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
   * Normaliza el conteo de tokens específico del proveedor al formato canónico.
   * Cada proveedor nombra los campos distinto, así que las subclases lo sobreescriben.
   * Los tokens de imagen NO se reportan aparte: todos los proveedores los suman
   * dentro del conteo de prompt/input, así que ya vienen incluidos en `input`.
   * @param {any} _rawUsage - Objeto de usage crudo de la respuesta del proveedor
   * @returns {{input:number, output:number, cached:number, reasoning:number, total:number, raw:object}|null}
   */
  normalizeUsage(_rawUsage) {
    return null;
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
