import { randomUUID } from 'node:crypto';
import { MCPclient } from './mcpClient.js';
import { LLMFactory } from './handlers/llmFactory.js';
import { chatLogger } from '../../common/logger.js';
import { LLM, MCPenable } from '../../config/config.js';
import { TurnContext } from './turnContext.js';
import { ChatHistoryManager } from './chatHistoryManager.js';
import { emitAssistantError, emitAssistantMessage } from './chatEvents.js';
import { getContextParams, removeSubAgent } from './subAgentRegistry.js';

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
   * Procesa un mensaje con mutex por chatId.
   * Concurrent requests for the same chatId queue sequentially.
   * @param {string} chatId - ID de la conversación
   * @param {string} message - Mensaje del usuario
   * @param {Object} options - Opciones adicionales
   * @param {Array<string>} options.allowedTools - Lista de herramientas permitidas (null = todas)
   * @returns {Promise<Object>} Respuesta final
   */
  static async processMessage(chatId, message, options = {}) {
    const prev = chatLocks.get(chatId) || Promise.resolve();
    let resolve;
    const lock = new Promise((r) => {
      resolve = r;
    });
    chatLocks.set(chatId, lock);

    try {
      await prev;
      return await this._processMessage(chatId, message, options);
    } finally {
      resolve();
      // Clean up if no one else is queued behind us
      if (chatLocks.get(chatId) === lock) {
        chatLocks.delete(chatId);
      }
    }
  }

  /**
   * Internal: processes a message (called under chatId mutex).
   *
   * Orchestration only: ask for context, persist the user turn, call the LLM,
   * persist what comes back. Every "figure out what this turn needs" step
   * lives in TurnContext.
   */
  static async _processMessage(chatId, message, options = {}) {
    // Groups every LLM request triggered by this user message (initial call + tool loop
    // iterations) so the real cost of one turn is a single GROUP BY away.
    const turnId = randomUUID();
    this._logIncomingMessage(chatId, message);

    const ctx = await TurnContext.build(chatId, {
      allowedTools: options.allowedTools ?? null,
      getTools: (allowed) => this.getToolsForProvider(allowed),
      llmHandler,
    });

    try {
      await ChatHistoryManager.addMessage(chatId, 'user', { role: 'user', content: message });

      // historySnapshot was taken BEFORE the user message, so it serves as
      // full context on the no-session fallback path.
      const result = await llmHandler.processMessage(message, ctx.tools, ctx.historySnapshot, {
        sessionId: ctx.sessionId,
        instructions: ctx.systemInstructions,
        agent: ctx.agent,
      });

      await this._persistTurnResult(chatId, result, ctx, turnId);

      chatLogger.info('✓ Procesamiento completado para chat:', chatId);
      return llmHandler.normalizeResponse(result);
    } catch (error) {
      chatLogger.error('Error procesando mensaje:', error);

      // Let the handler handle session-related errors
      await llmHandler.handleSessionError(chatId, error, ctx.persistence);
      emitAssistantError(chatId, error.userMessage || `Error: ${error.message}`);

      throw error;
    }
  }

  /**
   * Logs the incoming user message, collapsing multipart content to a summary
   * so a base64 image never lands in the logs.
   */
  static _logIncomingMessage(chatId, message) {
    const preview = Array.isArray(message)
      ? `[${message.length} blocks: ${message.map((b) => b.type).join(', ')}]`
      : `"${message.substring(0, 40)}${message.length > 40 ? '...' : ''}"`;
    chatLogger.info(`📨 Chat: ${chatId} | Mensaje: ${preview}`);
  }

  /**
   * Records usage, reconciles session state, persists every output part and
   * kicks off the tool loop when the model asked for tools.
   */
  static async _persistTurnResult(chatId, result, ctx, turnId) {
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
      phase: 'initial',
      iteration: 0,
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

    let toolCallsFlag = false;
    for (const res of output) {
      if (res.type === 'function_call' || res.type === 'tool_call') {
        toolCallsFlag = true;
      }
      const chatItem = await ChatHistoryManager.addMessage(chatId, 'assistant', res, responseId);
      this.emitAssistantMessage(chatItem);
    }

    if (toolCallsFlag) {
      this.runToolLoop(output, chatId, ctx, { turnId });
    }
  }

  /**
   * Fire-and-forget tool loop: the caller's turn returns as soon as the first
   * response is persisted, and the loop keeps streaming over the EventBus.
   *
   * @param {Array} output - Response parts containing the tool calls
   * @param {string} chatId
   * @param {TurnContext|object} ctx - Needs { tools, sessionId, systemInstructions, allowedTools, agent }
   * @param {object} usageCtx - Token accounting context, { turnId }
   */
  static runToolLoop(output, chatId, ctx, usageCtx = {}) {
    return this.handleToolCallsLoop(
      output,
      ctx.tools,
      chatId,
      ctx.sessionId,
      ctx.systemInstructions,
      ctx.allowedTools,
      ctx.agent,
      usageCtx
    ).catch((error) => {
      chatLogger.error(`[ToolLoop: ${chatId}] Error in tool calls loop:`, error);
      emitAssistantError(chatId, `Error processing tool results: ${error.message}`);
    });
  }

  /**
   * Maneja el loop de llamadas a herramientas
   * @param {Array} response - Initial response with tool calls
   * @param {Array} _tools - Available tools
   * @param {string} chatId - Chat identifier
   * @param {string} sessionId - Provider session ID for persistent state
   * @param {string} systemInstructions - System instructions to re-send
   * @param {Array<string>} allowedTools - List of allowed tool names (null = all)
   * @param {Object} agent - Full agent definition from resolveAgentForChat
   * @param {Object} usageCtx - Token accounting context, { turnId }. A fresh turnId is
   *   generated when absent (e.g. a loop resumed from subAgentManager).
   */
  static async handleToolCallsLoop(
    response,
    _tools,
    chatId,
    sessionId,
    systemInstructions,
    allowedTools = null,
    agent = null,
    usageCtx = {}
  ) {
    const turnId = usageCtx.turnId || randomUUID();
    let isToolCalling = true;
    let iterations = 0;
    chatLogger.debug('Starting tool calls loop...');
    let currentResponse = response;

    const maxIter = agent?.capability === 'high' ? maxIterations_planner : maxIterations;

    while (isToolCalling && iterations < maxIter) {
      iterations++;
      chatLogger.debug(`Iteration ${iterations} - Processing tools...`);
      isToolCalling = true;
      const toolResults = await this.executeToolCalls(currentResponse, chatId);

      // Check if this is the last iteration - force final response
      const isLastIteration = iterations >= maxIter;

      // Continue with the same session
      const result = await this.continueAfterTools(
        toolResults,
        chatId,
        sessionId,
        systemInstructions,
        allowedTools, //isLastIteration ? [] : allowedTools, // before  No tools on last iteration , now we allow tools on last iteration to let the LLM finish naturally
        isLastIteration, // forceFinish flag
        agent,
        false,
        { turnId, phase: 'tool_loop', iteration: iterations }
      );
      currentResponse = result.output;

      // Update stored responseId for tracking (conversation persists automatically)
      if (result.responseId) {
        await ChatHistoryManager.updateChatMetadata(chatId, { responseId: result.responseId });
      }

      if (isLastIteration) {
        chatLogger.warn('Maximum iterations reached - forced final response without tools');
        break;
      }

      isToolCalling = currentResponse.some(
        (content) => content.type === 'function_call' || content.type === 'tool_call'
      );
    }
    chatLogger.debug('Tool calls loop finished.');

    return currentResponse;
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
   * Continúa la conversación después de ejecutar herramientas
   * @param {Array} toolResults - Results from tool executions
   * @param {string} chatId - Chat identifier
   * @param {string} sessionId - Provider session ID for persistent state
   * @param {string} systemInstructions - System instructions to re-send
   * @param {Array<string>} allowedTools - List of allowed tool names (null = all, [] = none for final response)
   * @param {boolean} forceFinish - If true, adds a system message forcing final response
   * @param {Object} agent - Full agent definition from resolveAgentForChat
   * @param {boolean} skipPersist - Skip persisting tool results already stored
   * @param {Object} usageCtx - Token accounting context, { turnId, phase, iteration }
   * @returns {Promise<{output: Array, responseId: string}>} Response output and new responseId
   */
  static async continueAfterTools(
    toolResults,
    chatId,
    sessionId,
    systemInstructions,
    allowedTools = null,
    forceFinish = false,
    agent = null,
    skipPersist = false,
    usageCtx = {}
  ) {
    // Load history only if needed for fallback (no session)
    const conversationHistory = sessionId ? [] : await ChatHistoryManager.loadHistory(chatId);

    // Get tools filtered by allowedTools (empty array = no tools for forced text response)
    const tools = this.getToolsForProvider(allowedTools);

    // Persist tool results in DB before sending to LLM (single write point)
    // skipPersist=true when the result is already stored (e.g. returnMissionPlanXYZ replaced it in-place)
    if (!skipPersist) {
      for (const res of toolResults) {
        const chatItem = await ChatHistoryManager.addMessage(chatId, 'assistant', res);
        this.emitAssistantMessage(chatItem);
      }
    }

    // Continue conversation with tool outputs
    // When skipPersist=true the tool result is already in DB history — don't pass it as
    // toolOutputs or Gemini will receive it twice (once from history, once as functionResponse).
    const result = await llmHandler.processMessage(
      null, // No new user message
      tools,
      conversationHistory,
      {
        sessionId,
        instructions: systemInstructions,
        toolOutputs: skipPersist ? null : toolResults,
        forceFinish, // Signal to add final response message
        agent,
      }
    );

    const { output, responseId } = result;

    await ChatHistoryManager.recordUsage({
      chatId,
      turnId: usageCtx.turnId || randomUUID(),
      usage: result.usage,
      responseId,
      provider: llmHandler.getProviderName(),
      model: result.model,
      agent: agent?.name ?? null,
      phase: usageCtx.phase || 'tool_loop',
      iteration: usageCtx.iteration ?? 0,
    });

    // Store assistant messages with new responseId and emit events
    for (const res of output) {
      const chatItem = await ChatHistoryManager.addMessage(chatId, 'assistant', res, responseId);
      this.emitAssistantMessage(chatItem);
    }

    return { output, responseId };
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
   * @returns {Promise<Object>} Chat creado con id, name, createdAt
   */
  static async createChat(name = null) {
    try {
      const chat = await ChatHistoryManager.createChat(name);
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
