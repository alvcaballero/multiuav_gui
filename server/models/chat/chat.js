import { MCPclient } from './mcpClient.js';
import { LLMFactory } from './handlers/llmFactory.js';
import { chatLogger } from '../../common/logger.js';
import { LLM, MCPenable } from '../../config/config.js';
import { resolveAgentForChat } from './agents/index.js';
import { eventBus, EVENTS } from '../../common/eventBus.js';
import { ChatHistoryManager } from './chatHistoryManager.js';
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
    if (chatItem.from === 'assistant') {
      eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, chatItem);
    }
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
   * Internal: processes a message (called under chatId mutex)
   */
  static async _processMessage(chatId, message, options = {}) {
    const { allowedTools: optionsAllowedTools = null } = options;
    const msgPreview = Array.isArray(message)
      ? `[${message.length} blocks: ${message.map((b) => b.type).join(', ')}]`
      : `"${message.substring(0, 40)}${message.length > 40 ? '...' : ''}"`;
    chatLogger.info(`📨 Chat: ${chatId} | Mensaje: ${msgPreview}`);

    // Load history snapshot from DB (single source of truth)
    let historySnapshot;
    try {
      historySnapshot = await ChatHistoryManager.loadHistory(chatId);
      chatLogger.info(`📂 Loaded ${historySnapshot.length} messages from DB for chat: ${chatId}`);
    } catch (error) {
      chatLogger.error('Error loading history from DB:', error);
      historySnapshot = [];
    }

    const persistence = ChatHistoryManager.getSessionPersistence();

    const agent = await resolveAgentForChat(chatId);
    const agentProfile = agent.name;
    const allowedTools = optionsAllowedTools ?? agent.allowedTools;
    chatLogger.debug(`Using agent '${agentProfile}' with tools: ${allowedTools ? allowedTools.join(', ') : 'all'}`);

    try {
      // Get tools from MCP client, filtered by allowedTools
      const tools = this.getToolsForProvider(allowedTools);

      // Forked chats have copied history that the LLM provider doesn't know about.
      // On the first turn of a fork, bypass the session so the full history snapshot
      // is sent to the provider (CASE 3 in processMessage). After that turn the
      // provider creates a real session seeded with the complete context.
      const chatMeta = await ChatHistoryManager.getChatMetadata(chatId);
      const isUnseededFork = !!(chatMeta.forkedFrom && !chatMeta.sessionId);

      const sessionId = isUnseededFork ? null : await llmHandler.ensureSession(chatId, persistence);
      if (isUnseededFork) {
        chatLogger.info(
          `[fork] First turn of forked chat ${chatId} — using full-history path to seed provider context`
        );
      }

      // Build system instructions (needed for first message of conversation)
      let systemInstructions = null;
      if (agent.systemPrompt) {
        systemInstructions = `${agent.systemPrompt}\n\n---\nSession context:\n- chat_id: ${chatId}`;
      }

      // Add system prompt to history if first message (for record keeping)
      if (historySnapshot.length === 0 && systemInstructions) {
        const systemItem = await ChatHistoryManager.addMessage(chatId, 'system', {
          role: 'system',
          content: systemInstructions,
        });
        historySnapshot.push(systemItem);
      } else {
        // Recover system prompt from history to maintain consistency (e.g. after server restarts)
        const systemMsg = historySnapshot.find((item) => (item.message || item).role === 'system');
        if (systemMsg) {
          systemInstructions = (systemMsg.message || systemMsg).content;
        }
      }

      // Agregar el mensaje del usuario a DB
      await ChatHistoryManager.addMessage(chatId, 'user', { role: 'user', content: message });

      // Call LLM with session support (or fallback to full history)
      // historySnapshot was taken BEFORE user message, so it serves as context for fallback
      const result = await llmHandler.processMessage(message, tools, historySnapshot, {
        sessionId,
        instructions: systemInstructions,
        agent,
      });

      // Extract response data from result
      const { output, responseId, model, sessionCleared } = result;
      chatLogger.info(`✓ Parsed ${output.length} output parts from LLM response`);

      // Let the handler recover from session errors (e.g., recreate expired conversation)
      if (sessionCleared) {
        await llmHandler.handleSessionError(chatId, result, persistence);
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

      // Handle tool calls with the current sessionId
      // Pass allowedTools and agentProfile to maintain consistency across iterations
      if (toolCallsFlag) {
        this.handleToolCallsLoop(output, tools, chatId, sessionId, systemInstructions, allowedTools, agent).catch(
          (error) => {
            chatLogger.error(`[ToolLoop: ${chatId}] Error in tool calls loop:`, error);
            eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, {
              chatId,
              from: 'assistant',
              timestamp: new Date().toISOString(),
              message: {
                role: 'assistant',
                content: `Error processing tool results: ${error.message}`,
                type: 'text',
                status: 'error',
              },
            });
          }
        );
      }

      chatLogger.info('✓ Procesamiento completado para chat:', chatId);

      return llmHandler.normalizeResponse(result);
    } catch (error) {
      chatLogger.error('Error procesando mensaje:', error);

      // Let the handler handle session-related errors
      await llmHandler.handleSessionError(chatId, error, persistence);

      eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, {
        chatId,
        message: {
          role: 'assistant',
          content: error.userMessage || `Error: ${error.message}`,
          type: 'text',
          status: 'error',
        },
        timestamp: new Date().toISOString(),
      });

      throw error;
    }
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
   */
  static async handleToolCallsLoop(
    response,
    _tools,
    chatId,
    sessionId,
    systemInstructions,
    allowedTools = null,
    agent = null
  ) {
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
        agent
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
    skipPersist = false
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
