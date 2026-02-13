import { encode } from '@toon-format/toon';
import { MCPclient } from './mcpClient.js';
import { LLMFactory } from './llmFactory.js';
import { chatLogger } from '../../common/logger.js';
import { LLM, MCPenable } from '../../config/config.js';
import { SystemPrompts } from './prompts/index.js';
import { eventBus, EVENTS } from '../../common/eventBus.js';
import { ChatHistoryManager } from './chatHistoryManager.js';
import { convertMissionBriefingToXYZ, convertMissionXYZToLatLong } from './coordinateConverter.js';
import { missionController } from '../../controllers/mission.js';

let mcpClient = null;
let llmHandler = null;
const maxIterations = 6; // Prevenir loops infinitos

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
  static _emitAssistantMessage(chatItem) {
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
    chatLogger.info(`📨 Chat: ${chatId} | Mensaje: "${message.substring(0, 40)}${message.length > 40 ? '...' : ''}"`);

    // Load history snapshot from DB (single source of truth)
    let historySnapshot;
    try {
      historySnapshot = await ChatHistoryManager.loadHistory(chatId);
      chatLogger.info(`📂 Loaded ${historySnapshot.length} messages from DB for chat: ${chatId}`);
    } catch (error) {
      chatLogger.error('Error loading history from DB:', error);
      historySnapshot = [];
    }

    // Determinar allowedTools: usar opciones si se pasan, sino cargar de metadata
    let allowedTools = optionsAllowedTools;
    if (optionsAllowedTools !== null) {
      // Si se pasan allowedTools en opciones, guardarlas en metadata para consistencia
      await ChatHistoryManager.setAllowedTools(chatId, optionsAllowedTools);
    } else {
      // Si no se pasan, intentar cargar de metadata (para mantener consistencia en el chat)
      const storedAllowedTools = await ChatHistoryManager.getAllowedTools(chatId);
      if (storedAllowedTools !== undefined) {
        allowedTools = storedAllowedTools;
        chatLogger.debug(
          `Using stored allowedTools for chat ${chatId}: ${allowedTools ? allowedTools.join(', ') : 'all'}`
        );
      }
    }

    const persistence = ChatHistoryManager.getSessionPersistence();

    // Resolve agent profile from metadata (set by buildMissionPlanXYZ or defaults to 'default')
    const agentProfile = await ChatHistoryManager.getAgentProfile(chatId);

    try {
      // Get tools from MCP client, filtered by allowedTools
      const tools = this.getToolsForProvider(allowedTools);

      // Let the handler manage its own session (OpenAI creates conversations, others may no-op)
      const sessionId = await llmHandler.ensureSession(chatId, persistence);

      // Build system instructions (needed for first message of conversation)
      let systemInstructions = null;
      if (SystemPrompts.main) {
        systemInstructions = `${SystemPrompts.main}\n\n---\nSession context:\n- chat_id: ${chatId}`;
      }

      // Add system prompt to history if first message (for record keeping)
      if (historySnapshot.length === 0 && systemInstructions) {
        const systemItem = await ChatHistoryManager.addMessage(chatId, 'system', {
          role: 'system',
          content: systemInstructions,
        });
        historySnapshot.push(systemItem);
      }

      // Agregar el mensaje del usuario a DB
      await ChatHistoryManager.addMessage(chatId, 'user', { role: 'user', content: message });

      // Call LLM with session support (or fallback to full history)
      // historySnapshot was taken BEFORE user message, so it serves as context for fallback
      const result = await llmHandler.processMessage(message, tools, historySnapshot, {
        sessionId,
        instructions: systemInstructions,
        agentProfile,
      });

      // Extract response data from result
      const { output, responseId, model, sessionCleared } = result;

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
        this._emitAssistantMessage(chatItem);
      }

      // Handle tool calls with the current sessionId
      // Pass allowedTools and agentProfile to maintain consistency across iterations
      if (toolCallsFlag) {
        this.handleToolCallsLoop(output, tools, chatId, sessionId, systemInstructions, allowedTools, agentProfile);
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
   * @param {string} agentProfile - Agent profile name for model/reasoning selection
   */
  static async handleToolCallsLoop(
    response,
    _tools,
    chatId,
    sessionId,
    systemInstructions,
    allowedTools = null,
    agentProfile = 'default'
  ) {
    let isToolCalling = true;
    let iterations = 0;
    chatLogger.debug('Starting tool calls loop...');
    let currentResponse = response;

    while (isToolCalling && iterations < maxIterations) {
      iterations++;
      chatLogger.debug(`Iteration ${iterations} - Processing tools...`);
      isToolCalling = true;
      const toolResults = await this.executeToolCalls(currentResponse, chatId);

      // Check if this is the last iteration - force final response
      const isLastIteration = iterations >= maxIterations;

      // Continue with the same session
      const result = await this.continueAfterTools(
        toolResults,
        chatId,
        sessionId,
        systemInstructions,
        isLastIteration ? [] : allowedTools, // No tools on last iteration to force text response
        isLastIteration, // forceFinish flag
        agentProfile
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

    for (const toolCall of toolCalls) {
      if (toolCall.type == 'function_call' || toolCall.type == 'tool_call') {
        const result = await llmHandler.handleToolCall(toolCall, async (name, args) => {
          return await mcpClient.executeTool(name, args);
        });
        results.push(result);
      }
    }

    // Store tool results in DB and emit events
    for (const res of results) {
      const chatItem = await ChatHistoryManager.addMessage(chatId, 'assistant', res);
      this._emitAssistantMessage(chatItem);
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
   * @param {string} agentProfile - Agent profile name for model/reasoning selection
   * @returns {Promise<{output: Array, responseId: string}>} Response output and new responseId
   */
  static async continueAfterTools(
    toolResults,
    chatId,
    sessionId,
    systemInstructions,
    allowedTools = null,
    forceFinish = false,
    agentProfile = 'default'
  ) {
    // Get tools filtered by allowedTools (empty array = no tools for forced text response)
    const tools = this.getToolsForProvider(allowedTools);

    // Load history only if needed for fallback (no session)
    const conversationHistory = sessionId ? [] : await ChatHistoryManager.loadHistory(chatId);

    // Continue conversation with tool outputs
    const result = await llmHandler.processMessage(
      null, // No new user message
      tools,
      conversationHistory,
      {
        sessionId,
        instructions: systemInstructions,
        toolOutputs: toolResults, // Pass tool outputs to be sent in input
        forceFinish, // Signal to add final response message
        agentProfile,
      }
    );

    const { output, responseId } = result;

    // Store assistant messages with new responseId and emit events
    for (const res of output) {
      const chatItem = await ChatHistoryManager.addMessage(chatId, 'assistant', res, responseId);
      this._emitAssistantMessage(chatItem);
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
   * Obtiene el historial de conversación para un chat específico
   * @param {string} chatId - ID del chat
   */
  static async getHistory(chatId) {
    try {
      return await ChatHistoryManager.loadHistory(chatId);
    } catch (error) {
      chatLogger.error('Error loading history from DB:', error);
      return [];
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

  /**
   * Processes a mission briefing from the main chat and creates a secondary background chat
   * for mission planning in local XYZ coordinates.
   *
   * Flow:
   * 1. Receives mission briefing from main chat (with geodetic coordinates)
   * 2. Converts coordinates to local XYZ (ENU: East/North/Up in meters)
   * 3. Creates a secondary chat dedicated to mission planning
   * 4. Starts background processing with specialized mission planning prompt
   * 5. Returns immediately - secondary chat processes asynchronously
   *
   * @param {Object} missionBriefing - Mission briefing from main chat (filteredMissionSchema structure)
   * @param {string} missionBriefing.chat_id - ID of the main chat that initiated this request
   * @param {Array} missionBriefing.devices - Available devices with lat/lng positions
   * @param {Array} missionBriefing.inspection_elements - Elements to inspect with lat/lng positions
   * @param {Object} missionBriefing.mission_requeriments - Mission requirements and constraints
   * @param {Object} missionBriefing.user_context - Original user request context
   * @returns {Promise<Object>} Object with secondaryChatId and converted mission data
   */
  static async buildMissionPlanXYZ(missionBriefing) {
    const mainChatId = missionBriefing.chat_id;
    chatLogger.info(`[MainChat: ${mainChatId}] Starting mission plan XYZ generation`);

    // ═══════════════════════════════════════════════════════════════════
    // STEP 1: Convert geodetic coordinates to local XYZ (ENU)
    // ═══════════════════════════════════════════════════════════════════
    const missionDataXYZ = convertMissionBriefingToXYZ(missionBriefing);
    const { origin_global } = missionDataXYZ;

    // ═══════════════════════════════════════════════════════════════════
    // STEP 2: Create secondary chat for background mission planning
    // ═══════════════════════════════════════════════════════════════════
    const secondaryChat = await this.createChat(`MP-XYZ-${mainChatId}`);
    const secondaryChatId = secondaryChat.id;
    chatLogger.info(`[MainChat: ${mainChatId}] Secondary chat XYZ: ${secondaryChatId}`);

    // Set allowed tools and agent profile in metadata for consistent config across all messages
    const missionPlanningTools = ['Show_mission_xyz_to_user', 'validate_mission_collisions'];
    await ChatHistoryManager.setAllowedTools(secondaryChatId, missionPlanningTools);
    await ChatHistoryManager.setAgentProfile(secondaryChatId, 'planner');

    // ═══════════════════════════════════════════════════════════════════
    // STEP 3: Configure secondary chat with mission-specific system prompt
    // ═══════════════════════════════════════════════════════════════════
    const systemPromptContent = `${SystemPrompts.mission_build_xyz}
---
Session context:
- main_chat_id: ${mainChatId}
- secondary_chat_id: ${secondaryChatId}
- coordinate_system: local XYZ (ENU - East/North/Up in meters)
- yaw_reference: Angle in degrees, 0 degrees = North (+Y), 90° = East (+X), ±180° = South (-Y), -90° = West (-X). Range: [-180°, 180°]`;

    await ChatHistoryManager.addMessage(secondaryChatId, 'system', { role: 'system', content: systemPromptContent });

    // ═══════════════════════════════════════════════════════════════════
    // STEP 4: Build the mission planning request message
    // ═══════════════════════════════════════════════════════════════════
    const userMessage = `Execute the MISSION PLANNING SEQUENCE with this data for ${missionDataXYZ.user_context.user_request} :

## Origin (Global Reference  metadata) ${encode(origin_global)}
## Devices Information
${encode(missionDataXYZ.drone_information)}
## Elements to Inspect
${encode(missionDataXYZ.target_elements)}
## group Information
${encode(missionDataXYZ.group_information)}
## obstacles Information
${encode(missionDataXYZ.obstacle_elements)}
## Mission Requirements
${encode(missionDataXYZ.mission_requirements)}`;

    // ═══════════════════════════════════════════════════════════════════
    // STEP 5: Start background processing (non-blocking)
    // ═══════════════════════════════════════════════════════════════════
    // processMessage handles LLM interaction and tool calls asynchronously
    // allowedTools and agentProfile already set in metadata — will be loaded automatically
    this.processMessage(secondaryChatId, userMessage).catch((error) => {
      chatLogger.error(`[MainChat: ${mainChatId}] Background mission planning failed:`, error);
      eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, {
        chatId: mainChatId,
        from: 'assistant',
        timestamp: new Date().toISOString(),
        message: {
          role: 'assistant',
          content: `Error en planificación de misión: ${error.message}`,
          type: 'text',
          status: 'error',
        },
      });
    });

    chatLogger.info(`[MainChat: ${mainChatId}] Background processing started in secondary chat: ${secondaryChatId}`);

    // Return immediately - secondary chat continues processing in background
    return { secondaryChatId, msg: 'Mission is processing.' };
  }
}
