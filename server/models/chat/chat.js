import { randomUUID } from 'node:crypto';
import { MCPclient } from './mcpClient.js';
import { LLMFactory } from './handlers/llmFactory.js';
import { chatLogger } from '../../common/logger.js';
import { LLM, MCPenable } from '../../config/config.js';
import { TurnContext } from './turnContext.js';
import { ChatHistoryManager } from './chatHistoryManager.js';
import { emitAssistantError, emitAssistantMessage, emitChatBusy } from './chatEvents.js';
import { getContextParams, removeSubAgent } from './subAgentRegistry.js';
import { forceFinishItem } from './handlers/baseLLMhandler.js';

let mcpClient = null;
let llmHandler = null;

const maxIterations = 25; // Prevenir loops infinitos
const maxIterations_planner = 18; // Prevenir loops infinitos

// Per-chatId mutex: ensures only one processMessage runs at a time per chat.
// Concurrent requests for the same chatId queue behind the active one.
const chatLocks = new Map();

export class MessageOrchestrator {
  static initializeLLMProvider(provider, apiKey) {
    if (LLM && !llmHandler) {
      llmHandler = LLMFactory.createHandler(provider, apiKey);
      llmHandler
        .initialize()
        .then(() => {
          chatLogger.info('LLM Handler initialized successfully.');
        })
        .catch((error) => {
          chatLogger.error('Error initializing LLM Handler:', error);
          llmHandler = null;
        });
      chatLogger.info('LLM Provider initialized:', { provider });
    }
    if (MCPenable && !mcpClient) {
      mcpClient = new MCPclient();
      mcpClient
        .connect()
        .then(() => {
          chatLogger.info('MCP Client connected successfully.');
        })
        .catch((error) => {
          chatLogger.error('Error connecting MCP Client:', error);
        });
    } else if (!MCPenable) {
      chatLogger.info('MCP is disabled, using only LLM provider.');
    }
  }

  /**
   * Verifica si el orquestador está listo para procesar mensajes
   * @returns {boolean}
   */

  static isReady() {
    return llmHandler !== null && llmHandler !== undefined && llmHandler.initialized === true;
  }

  /**
   * Emits EventBus event for assistant messages (WebSocket broadcast to clients)
   * @param {object} chatItem - The chat item to potentially emit
   */
  static emitAssistantMessage(chatItem) {
    emitAssistantMessage(chatItem);
  }

  /**
   * Normalizes whatever a caller passes as the turn input into the canonical
   * `{kind, ...}` shape. Plain strings and multipart arrays are user messages,
   * which keeps every existing `processMessage(chatId, "text")` call working.
   *
   * @param {string|Array|object} input
   * @returns {{kind: string}} Canonical turn input
   */
  static _normalizeInput(input) {
    if (typeof input === 'string' || Array.isArray(input)) {
      return { kind: 'user', content: input };
    }
    if (input && typeof input === 'object' && input.kind) return input;
    throw new Error('processMessage: input must be a string, a content array, or a {kind} object');
  }

  /**
   * Procesa un turno con mutex por chatId.
   * Concurrent requests for the same chatId queue sequentially.
   *
   * The lock is held for the WHOLE turn, tool loop included. The turn itself is
   * fire-and-forget past the first LLM response (the caller gets that response
   * while the loop keeps streaming over the EventBus), so the lock cannot be
   * released in this function's `finally` — that would free the chat while the
   * loop is still writing to it. `_runTurn` releases it when the recursion ends.
   *
   * @param {string} chatId - ID de la conversación
   * @param {string|Array|object} input - User message, or a `{kind, ...}` turn input
   * @param {Object} options - Opciones adicionales
   * @param {Array<string>} options.allowedTools - Lista de herramientas permitidas (null = todas)
   * @returns {Promise<Object>} Respuesta final
   */
  static async processMessage(chatId, input, options = {}) {
    const turnInput = this._normalizeInput(input);

    const prev = chatLocks.get(chatId) || Promise.resolve();
    let resolve;
    const lock = new Promise((r) => {
      resolve = r;
    });
    chatLocks.set(chatId, lock);

    const release = () => {
      resolve();
      // Clean up if no one else is queued behind us
      if (chatLocks.get(chatId) === lock) {
        chatLocks.delete(chatId);
      }
      emitChatBusy(chatId, false);
    };

    await prev;
    emitChatBusy(chatId, true);

    try {
      return await this._startTurn(chatId, turnInput, options, release);
    } catch (error) {
      release();
      throw error;
    }
  }

  /**
   * Internal: runs the first turn (called under chatId mutex).
   *
   * Orchestration only: ask for context, run the turn, hand the caller the first
   * response. If the model asked for tools, `_runTurn` recursed into the loop
   * before returning here, and `release` has already fired.
   */
  static async _startTurn(chatId, turnInput, options = {}, release = () => {}) {
    // Groups every LLM request triggered by this input (initial call + tool loop
    // iterations) so the real cost of one turn is a single GROUP BY away.
    const turnId = randomUUID();
    this._logIncomingMessage(chatId, turnInput);

    // A subagent answer is appended to a conversation the provider's session
    // never saw, so the session is dropped to force the full-history path.
    if (turnInput.kind === 'subagent_result') {
      await ChatHistoryManager.clearSession(chatId);
    }

    const ctx = await TurnContext.build(chatId, {
      allowedTools: options.allowedTools ?? null,
      getTools: (allowed) => this.getToolsForProvider(allowed),
      llmHandler,
    });

    try {
      const result = await this._runTurn(chatId, turnInput, ctx, {
        turnId,
        phase: 'initial',
        iteration: 0,
        release,
      });

      chatLogger.info('✓ Procesamiento completado para chat:', chatId);
      return llmHandler.normalizeResponse(result.raw);
    } catch (error) {
      chatLogger.error('Error procesando mensaje:', error);

      // Let the handler handle session-related errors
      await llmHandler.handleSessionError(chatId, error, ctx.persistence);
      emitAssistantError(chatId, error.userMessage || `Error: ${error.message}`);

      throw error;
    }
  }

  /**
   * One turn: persist the input, call the LLM, persist the output — then, if the
   * model asked for tools, run them and recurse with their results.
   *
   * This single function replaces the old `_processMessage` / `continueAfterTools`
   * pair, which were the same three steps with different inputs. What varies is
   * ONLY how the input reaches the provider, and that lives in `_applyInput`.
   *
   * The recursion carries `iteration` as a parameter rather than tracking it per
   * chat: the cap protects ONE turn from running away, and a turn already has an
   * identity (`turnId`). A fresh user message is a fresh turn, so it starts at 0
   * with nothing to reset.
   *
   * @param {string} chatId
   * @param {object} turnInput - `{kind, ...}` canonical input
   * @param {TurnContext} ctx
   * @param {object} meta - `{turnId, phase, iteration, release}`
   * @returns {Promise<{output: Array, responseId: string, raw: object}>}
   */
  static async _runTurn(chatId, turnInput, ctx, meta) {
    const { turnId, phase, iteration, release } = meta;

    // Read fresh HERE — before `_applyInput` persists this turn's own input below —
    // so it never contains what this turn is about to write. That write travels
    // separately via providerInput; reading it back from a later DB read would
    // send it to the provider twice. (With a live session the provider keeps the
    // conversation itself, so most handlers ignore this in the happy path — it
    // only matters as their session-error fallback.)
    const history = await ChatHistoryManager.loadHistory(chatId);

    const providerInput = await this._applyInput(chatId, turnInput);

    // forceFinish rides inside providerInput.items instead of a separate flag —
    // each handler picks the 'directive' item out and inserts it wherever its
    // API requires. Only a tool_output turn ever sets forceFinish (see
    // _continueWithTools), so providerInput is always the 'tool_output' shape here.
    if (turnInput.forceFinish && providerInput?.type === 'tool_output') {
      providerInput.items = [...providerInput.items, forceFinishItem()];
    }

    const result = await llmHandler.processMessage(providerInput, ctx.tools, history, {
      sessionId: ctx.sessionId,
      instructions: ctx.systemInstructions,
      agent: ctx.agent,
    });

    const { output, responseId } = await this._persistTurnResult(chatId, result, ctx, {
      turnId,
      phase,
      iteration,
    });

    const hasToolCalls = output.some((res) => res.type === 'function_call' || res.type === 'tool_call');
    if (!hasToolCalls) {
      release();
      return { output, responseId, raw: result };
    }

    // Fire-and-forget from here on: the caller gets the first response while the
    // loop keeps going and streams over the EventBus. The lock stays held until
    // the recursion bottoms out, so a queued message cannot interleave with it.
    this._continueWithTools(chatId, output, ctx, { turnId, iteration }).finally(release);

    return { output, responseId, raw: result };
  }

  /**
   * Executes the tool calls in `output` and recurses with their results.
   * Split out of `_runTurn` so the fire-and-forget boundary is explicit: this is
   * the part that outlives the caller's turn.
   */
  static async _continueWithTools(chatId, output, ctx, { turnId, iteration }) {
    const maxIter = ctx.agent?.capability === 'high' ? maxIterations_planner : maxIterations;
    const next = iteration + 1;
    const isLastIteration = next >= maxIter;

    if (isLastIteration) {
      chatLogger.warn(`[ToolLoop: ${chatId}] Maximum iterations reached - forcing final response`);
    }
    chatLogger.debug(`[ToolLoop: ${chatId}] Iteration ${next} - Processing tools...`);

    try {
      const results = await this.executeToolCalls(output, chatId);

      await this._runTurn(
        chatId,
        { kind: 'tool_output', results, forceFinish: isLastIteration },
        ctx,
        // The last iteration must not recurse again: `release` is a no-op here
        // because THIS call's `.finally(release)` already owns it.
        { turnId, phase: 'tool_loop', iteration: next, release: () => {} }
      );
    } catch (error) {
      chatLogger.error(`[ToolLoop: ${chatId}] Error in tool calls loop:`, error);
      emitAssistantError(chatId, `Error processing tool results: ${error.message}`);
    }
  }

  /**
   * Persists a turn's input and returns what the provider needs for it.
   *
   * Every kind writes its own input to history exactly once — that symmetry is
   * what removed the old `skipPersist` flag, which existed only because the
   * subagent path wrote its message somewhere else first.
   *
   * @returns {Promise<?{type: 'message', content: *}|{type: 'tool_output', items: Array}>} What
   *   `llmHandler.processMessage` receives as its first argument. `null` means "nothing new to
   *   send — continue purely from the persisted history".
   */
  static async _applyInput(chatId, turnInput) {
    switch (turnInput.kind) {
      case 'user':
        await ChatHistoryManager.addMessage(chatId, 'user', { role: 'user', content: turnInput.content });
        // The history _runTurn read is from BEFORE this message was persisted
        // above, so it serves as full context on the no-session fallback path.
        return { type: 'message', content: turnInput.content };

      case 'subagent_result': {
        const chatItem = await ChatHistoryManager.addMessage(chatId, 'subagent', turnInput.message);
        this.emitAssistantMessage(chatItem);
        // Sent as its own turn item (like tool_output), not replayed from
        // history: the parent's tool pair closed long ago, so there is no
        // pending call for it to answer — it's a fresh user-role message.
        return { type: 'subagent_result', message: turnInput.message };
      }

      case 'tool_output': {
        for (const res of turnInput.results) {
          const chatItem = await ChatHistoryManager.addMessage(chatId, 'assistant', res);
          this.emitAssistantMessage(chatItem);
        }
        return { type: 'tool_output', items: turnInput.results };
      }

      default:
        throw new Error(`_applyInput: unknown turn input kind "${turnInput.kind}"`);
    }
  }

  /**
   * Logs the incoming turn input, collapsing multipart content to a summary
   * so a base64 image never lands in the logs.
   */
  static _logIncomingMessage(chatId, turnInput) {
    const { kind, content } = turnInput;
    let preview;

    if (kind !== 'user') {
      preview = `[${kind}]`;
    } else if (Array.isArray(content)) {
      preview = `[${content.length} blocks: ${content.map((b) => b.type).join(', ')}]`;
    } else {
      preview = `"${String(content).substring(0, 40)}${String(content).length > 40 ? '...' : ''}"`;
    }

    chatLogger.info(`📨 Chat: ${chatId} | Mensaje: ${preview}`);
  }

  /**
   * Records usage, reconciles session state and persists every output part.
   *
   * Deciding what happens NEXT (tool loop or not) belongs to `_runTurn`; this
   * one only writes down what came back.
   *
   * @returns {Promise<{output: Array, responseId: string}>}
   */
  static async _persistTurnResult(chatId, result, ctx, { turnId, phase, iteration }) {
    const { output, responseId, model, sessionCleared, usage } = result;
    chatLogger.info(`✓ Parsed ${output.length} output parts from LLM response`);

    await ChatHistoryManager.recordUsage({
      chatId,
      turnId,
      usage,
      responseId,
      provider: llmHandler.getProviderName(),
      model,
      agent: ctx.agent.name,
      phase,
      iteration,
    });

    // Let the handler recover from session errors (e.g., recreate expired conversation)
    if (sessionCleared) {
      await llmHandler.handleSessionError(chatId, result, ctx.persistence);
    }
    // Store metadata for this chat
    else if (responseId) {
      await ChatHistoryManager.updateChatMetadata(chatId, {
        responseId,
        provider: llmHandler.getProviderName(),
        model,
      });
    }

    for (const res of output) {
      const chatItem = await ChatHistoryManager.addMessage(chatId, 'assistant', res, responseId);
      this.emitAssistantMessage(chatItem);
    }

    return { output, responseId };
  }

  /**
   * Ejecuta todas las llamadas a herramientas solicitadas por el LLM
   */
  static async executeToolCalls(toolCalls, chatId) {
    const results = [];
    const contextParams = getContextParams(chatId);
    const hasContext = Object.keys(contextParams).length > 0;

    for (const toolCall of toolCalls) {
      if (toolCall.type == 'function_call' || toolCall.type == 'tool_call') {
        const result = await llmHandler.handleToolCall(toolCall, async (name, args) => {
          // Fixed context params win over whatever the LLM passes, so it can't
          // override a subagent's injected context even if it hallucinates the same key.
          return await mcpClient.executeTool(name, hasContext ? { ...args, ...contextParams } : args);
        });
        results.push(result);
      }
    }

    return results;
  }

  /**
   * Obtiene las herramientas en el formato correcto para el proveedor actual
   * Filtra las herramientas basándose en allowedTools si se especifica
   * @param {Array<string>|null} allowedTools - Lista de herramientas permitidas (null = todas)
   * @returns {Array} Lista de herramientas (filtradas si corresponde)
   */
  static getToolsForProvider(allowedTools = null) {
    if (!mcpClient || !mcpClient.isReady()) {
      chatLogger.debug('No MCP tools available');
      return [];
    }

    const tools = mcpClient.getTools();

    if (!tools || tools.length === 0) {
      chatLogger.debug('No MCP tools available');
      return [];
    }

    // Si allowedTools es null o undefined, devolver todas las herramientas
    if (allowedTools === null || allowedTools === undefined) {
      return tools;
    }

    // Si allowedTools es un array vacío, no devolver herramientas (forzar respuesta de texto)
    if (Array.isArray(allowedTools) && allowedTools.length === 0) {
      chatLogger.debug('No tools allowed (empty allowedTools array)');
      return [];
    }

    // Filtrar herramientas basándose en allowedTools
    const filteredTools = tools.filter((tool) => allowedTools.includes(tool.name));
    chatLogger.debug(`Filtered tools: ${filteredTools.map((t) => t.name).join(', ')} (from ${tools.length} total)`);
    return filteredTools;
  }

  /**
   * Obtiene una página del historial de conversación para un chat específico,
   * la más reciente por defecto o la anterior a `before` (paginación por cursor).
   * @param {string} chatId - ID del chat
   * @param {object} options
   * @param {number} options.limit - Tamaño de página (default 100)
   * @param {string|null} options.before - Cursor ISO timestamp; trae mensajes estrictamente anteriores
   * @returns {Promise<{messages: Array, hasMore: boolean}>}
   */
  static async getHistory(chatId, { limit = 100, before = null } = {}) {
    try {
      // Fetch one extra row to know whether older messages remain, then drop it.
      const rows = await ChatHistoryManager.loadHistory(chatId, {
        all: true,
        limit: limit + 1,
        before,
        order: 'DESC',
      });
      const hasMore = rows.length > limit;
      return { messages: hasMore ? rows.slice(1) : rows, hasMore };
    } catch (error) {
      chatLogger.error('Error loading history from DB:', error);
      return { messages: [], hasMore: false };
    }
  }

  /**
   * Token usage of a chat: per-LLM-call detail, per-turn breakdown and totals.
   * One turn = one user message, which can span many LLM requests
   * (the initial call plus every tool loop iteration).
   * @param {string} chatId
   * @returns {Promise<{totals: object, turns: Array, requests: Array}>}
   */
  static async getUsage(chatId) {
    return ChatHistoryManager.getUsageForChat(chatId);
  }

  /**
   * Lista todos los chats disponibles
   */
  static async listChats() {
    try {
      const dbChats = await ChatHistoryManager.getAllChats();
      const chats = [];

      for (const dbChat of dbChats) {
        const messageCount = await ChatHistoryManager.getMessageCount(dbChat.id);
        chats.push({
          id: dbChat.id,
          name: dbChat.name,
          messageCount,
          lastUpdated: dbChat.updatedAt,
          createdAt: dbChat.createdAt,
          source: 'database',
        });
      }

      // Sort by createdAt descending (newest first)
      return chats.sort((a, b) => {
        const dateA = new Date(a.createdAt);
        const dateB = new Date(b.createdAt);
        return dateB - dateA;
      });
    } catch (error) {
      chatLogger.error('Error listing DB chats:', error);
      return [];
    }
  }

  /**
   * Elimina un chat completamente
   * @param {string} chatId - ID del chat a eliminar
   * @param {boolean} hardDelete - Si true, elimina permanentemente de la BD
   */
  static async deleteChat(chatId, hardDelete = false) {
    try {
      await ChatHistoryManager.deleteChat(chatId, hardDelete);
    } catch (error) {
      chatLogger.error('Error deleting chat from DB:', error);
    }
    removeSubAgent(chatId);
    chatLogger.info(`Chat deleted: ${chatId}`);
  }

  /**
   * Renombra un chat
   * @param {string} chatId - ID del chat
   * @param {string} name - Nuevo nombre
   */
  static async renameChat(chatId, name) {
    try {
      await ChatHistoryManager.updateChat(chatId, { name });
      return true;
    } catch (error) {
      chatLogger.error('Error renaming chat:', error);
      return false;
    }
  }

  /**
   * Fork a conversation up to (and including) a specific message timestamp.
   * @param {string} sourceChatId - Source chat ID
   * @param {string} upToTimestamp - ISO timestamp of the last message to include
   * @param {string} name - Optional name for the new chat
   * @returns {Promise<Object>} New chat with id, name, createdAt
   */
  static async forkChat(sourceChatId, upToTimestamp, name = null) {
    try {
      const chat = await ChatHistoryManager.forkChat(sourceChatId, upToTimestamp, name);
      chatLogger.info(`Chat forked: ${sourceChatId} → ${chat.id}`);
      return {
        id: chat.id,
        name: chat.name,
        createdAt: chat.createdAt,
      };
    } catch (error) {
      chatLogger.error('Error forking chat:', error);
      throw error;
    }
  }

  /**
   * Crea un nuevo chat y devuelve su ID
   * @param {string} name - Nombre opcional del chat
   * @param {Object} metadata - Metadata opcional inicial del chat
   * @returns {Promise<Object>} Chat creado con id, name, createdAt
   */
  static async createChat(name = null, metadata = {}) {
    try {
      const chat = await ChatHistoryManager.createChat(name, metadata);
      chatLogger.info(`Chat created: ${chat.id}`);
      return {
        id: chat.id,
        name: chat.name,
        createdAt: chat.createdAt,
      };
    } catch (error) {
      chatLogger.error('Error creating chat:', error);
      throw error;
    }
  }

  static async testMcpTool(toolName, toolArgs = {}) {
    if (!mcpClient || !mcpClient.isReady()) {
      throw new Error('MCP client not connected or not ready');
    }

    const availableTools = mcpClient.getTools().map((t) => t.name);
    if (!availableTools.includes(toolName)) {
      throw new Error(`Tool "${toolName}" not found. Available: ${availableTools.join(', ')}`);
    }

    chatLogger.info(`[testMcpTool] Executing tool: ${toolName}`);
    const result = await mcpClient.executeTool(toolName, toolArgs);
    chatLogger.info(`[testMcpTool] Tool "${toolName}" executed successfully`);
    return result;
  }
}
