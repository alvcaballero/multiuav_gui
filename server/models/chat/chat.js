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
import { Json } from 'sequelize/lib/utils';

let mcpClient = null;
let llmHandler = null;

const maxIterations = 6; // Prevenir loops infinitos
const maxIterations_planner = 18; // Prevenir loops infinitos

// Default allowed tools per agent profile.
// When processMessage receives no allowedTools and none are stored in metadata,
// this map determines which tools are available based on the agent profile.
// null = all tools (no filtering), [] = no tools, [...] = specific tools

const AGENT_DEFAULT_TOOLS = {
  default: [
    'get_devices',
    'get_fleet_telemetry',
    'get_registered_objects',
    'get_bases_with_assignments',
    'show_mission_to_user',
    'request_mission_plan',
    'load_mission_to_uav',
    'start_mission',
  ],
  planner: ['show_mission_xyz', 'validate_mission_collisions', 'mark_step_complete', 'complete_mission'],
};

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

    // Fallback: if no allowedTools from options or metadata, use agent profile defaults
    if (allowedTools === null && AGENT_DEFAULT_TOOLS[agentProfile]) {
      allowedTools = AGENT_DEFAULT_TOOLS[agentProfile];
      chatLogger.debug(`Using default tools for agent '${agentProfile}': ${allowedTools.join(', ')}`);
    }

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
        this.handleToolCallsLoop(
          output,
          tools,
          chatId,
          sessionId,
          systemInstructions,
          allowedTools,
          agentProfile
        ).catch((error) => {
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
        });
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

    let maxIter = agentProfile === 'planner' ? maxIterations_planner : maxIterations;

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
    agentProfile = 'default',
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
        this._emitAssistantMessage(chatItem);
      }
    }

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
      return await ChatHistoryManager.loadHistory(chatId,{all:true});
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

  static async convertMissionBriefingToXYZ(missionBriefing) {
    const missionDataXYZ = convertMissionBriefingToXYZ(missionBriefing);
    return {
      global_origin: missionDataXYZ.global_origin,
      devices_info: missionDataXYZ.drone_information,
      target_elements: missionDataXYZ.target_elements,
      group_information: missionDataXYZ.group_information,
      obstacle_elements: missionDataXYZ.obstacle_elements,
      mission_requirements: missionDataXYZ.mission_requirements,
      user_context: missionDataXYZ.user_context,
    };
  }

  /**

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
    const { global_origin } = missionDataXYZ;
    // ═══════════════════════════════════════════════════════════════════
    // STEP 4: Build the mission planning request message
    // ═══════════════════════════════════════════════════════════════════
    const userMessage = `Execute the MISSION PLANNING SEQUENCE with this data for ${missionDataXYZ.user_context.user_request} :

## global_origin_coordinates
${JSON.stringify(global_origin)}
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

    const secondaryChatId = `MP-XYZ-${mainChatId}`;
    this.subAgentPlannerChat(mainChatId, userMessage, missionDataXYZ);
    chatLogger.info(`[MainChat: ${mainChatId}] Background processing started in secondary chat: ${secondaryChatId}`);

    // Return immediately - secondary chat continues processing in background
    return { secondaryChatId, msg: 'Mission is processing.' };
  }

  static async subAgentPlannerChat(mainChatId, userMessage, missionDataXYZ) {
    const { global_origin } = missionDataXYZ;
    if (!userMessage) return null;
    if (!mainChatId) {
      mainChatId = `chat_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`;
      chatLogger.warn(`[subAgentPlannerChat] No mainChatId provided, generated: ${mainChatId}`);
    }

    // ═══════════════════════════════════════════════════════════════════
    // STEP 2: Create secondary chat for background mission planning
    // ═══════════════════════════════════════════════════════════════════
    const secondaryChat = await this.createChat(`MP-XYZ-${mainChatId}`);
    const secondaryChatId = secondaryChat.id;
    chatLogger.info(`[MainChat: ${mainChatId}] Secondary chat XYZ: ${secondaryChatId}`);

    // Set allowed tools and agent profile in metadata for consistent config across all messages
    await ChatHistoryManager.setAllowedTools(secondaryChatId, AGENT_DEFAULT_TOOLS['planner']);
    await ChatHistoryManager.setAgentProfile(secondaryChatId, 'planner');

    // ═══════════════════════════════════════════════════════════════════
    // STEP 3: Configure secondary chat with mission-specific system prompt
    // ═══════════════════════════════════════════════════════════════════
    const systemPromptContent = `${SystemPrompts.mission_build_xyz}
---
Session_context:
- main_chat_id: ${mainChatId}
- secondary_chat_id: ${secondaryChatId}
- global_origin_coordinates: ${JSON.stringify(global_origin)} (lat, lng in decimal degrees)
- coordinate_system: Cartesian coordinates (ENU - East/North/Up in meters)
- yaw_reference: Angle in degrees, 0 degrees = North (+Y), 90° = East (+X), ±180° = South (-Y), -90° = West (-X). Range: [-180°, 180°]

Mandatory: Maintain all the session context data accurately and unchanged the session.
`;

    await ChatHistoryManager.addMessage(secondaryChatId, 'system', { role: 'system', content: systemPromptContent });

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
  }

  /**
   * Converts a mission briefing from geodetic coordinates to local XYZ (ENU)
   * and returns the result as a plain JSON object — no LLM, no serialization.
   * Intended for external consumers (e.g. the mission evaluator) that need the
   * structured data directly instead of the text prompt built by buildMissionPlanXYZ.
   *
   * @param {Object} missionBriefing - filteredMissionSchema structure with geodetic coords
   * @returns {Promise<Object>} Converted mission briefing with XYZ coordinates + global_origin
   */
  static async convertMissionBriefingToXYZ(missionBriefing) {
    chatLogger.info('[convertMissionBriefingToXYZ] Converting mission briefing to XYZ');
    return convertMissionBriefingToXYZ(missionBriefing);
  }

  /**
   * Receives the real mission plan from the MCP `complete_mission` tool,
   * converts its waypoints from local XYZ (ENU) to geodetic coordinates,
   * replaces the placeholder tool_result for `request_mission_plan` in the
   * MAIN chat history, and re-runs processMessage so the main agent can
   * continue with the actual mission data.
   *
   * @param {Object} payload - Body from POST /chat/return_mission_plan_xyz
   * @param {string} payload.chat_id       - Main chat ID (from completeMissionSchema)
   * @param {string} payload.status        - 'valid' | 'error' | 'incomplete'
   * @param {string} payload.description   - Human-readable summary of the result
   * @param {Object} payload.missionDataXYZ - Mission in local XYZ coordinates (MissionSchemaXYZ)
   * @returns {Promise<Object>}
   */
  /**
   * Executes a MCP tool directly, bypassing the LLM.
   * Useful for debugging/testing tool availability and responses.
   *
   * @param {string} toolName - Name of the MCP tool to execute
   * @param {Object} toolArgs - Arguments to pass to the tool
   * @returns {Promise<Object>} Raw result from the MCP tool
   */
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

  static async returnMissionPlanXYZ({ chat_id, status, description, missionDataXYZ }) {
    chatLogger.info(`[returnMissionPlanXYZ] Received mission result for main chat: ${chat_id} | status: ${status}`);

    // ── 0. Verify the main chat exists ───────────────────────────────────────
    const chatExists = await ChatHistoryManager.chatExists(chat_id);
    if (!chatExists) {
      const err = new Error(`Chat not found: ${chat_id}`);
      err.statusCode = 404;
      throw err;
    }

    // ── 1. Convert waypoints XYZ → geodetic ─────────────────────────────────
    let missionGeodetic = null;
    try {
      missionGeodetic = convertMissionXYZToLatLong(missionDataXYZ);
      chatLogger.info(
        `[returnMissionPlanXYZ] Converted mission to geodetic. Routes: ${missionGeodetic?.route?.length ?? 0}`
      );
    } catch (err) {
      chatLogger.error('[returnMissionPlanXYZ] Coordinate conversion failed:', err);
      throw err;
    }

    // ── 2. Build the new output string ───────────────────────────────────────
    // Preserve the exact output format the MCP handlers produce:
    // a JSON string wrapping content[].text, same as other tool responses.
    const newOutput = JSON.stringify({
      content: [{ type: 'text', text: JSON.stringify({ status, description, mission: missionGeodetic }) }],
    });
    const newContent = `Mission plan result [${status}]: ${description}`;

    // ── 3. Hide placeholder, insert new message with real data ───────────────
    // Original message is marked hidden=true (UI sees it, LLM won't).
    // New message clones original messageData and only replaces `output`,
    // so call_id, type, name and all LLM-set fields are preserved.
    const hidden = await ChatHistoryManager.hideAndReplaceToolResult(
      chat_id,
      'request_mission_plan',
      newOutput,
      newContent
    );

    if (!hidden) {
      chatLogger.warn(
        `[returnMissionPlanXYZ] No request_mission_plan tool_result found in chat ${chat_id} — injecting as new message`
      );
      // Fallback: inject as a bare tool result message
      await ChatHistoryManager.addMessage(chat_id, 'assistant', {
        type: 'function_call_output',
        name: 'request_mission_plan',
        output: newOutput,
      });
    }

    // ── 4. Clear provider session so next call rebuilds context from the
    //      updated DB history (placeholder hidden, real result visible).
    await ChatHistoryManager.clearSession(chat_id);

    // ── 5. Continue the conversation with the real tool result.
    //      continueAfterTools avoids persisting a spurious user message.
    //      We need the actual messageData object for the LLM input, so we load
    //      the last non-hidden tool_result for request_mission_plan from DB.
    chatLogger.info(`[returnMissionPlanXYZ] Continuing main chat with real mission data: ${chat_id}`);

    const sessionId = await ChatHistoryManager.getSessionId(chat_id);
    const agentProfile = await ChatHistoryManager.getAgentProfile(chat_id);
    const allowedTools = await ChatHistoryManager.getAllowedTools(chat_id);

    const history = await ChatHistoryManager.loadHistory(chat_id);
    const systemMsg = history.find((item) => (item.message || item).role === 'system');
    const systemInstructions = systemMsg ? (systemMsg.message || systemMsg).content : null;

    // Get the real messageData that was just inserted (last visible tool_result for this tool)
    const realToolResult = history.findLast(
      (item) => item.message?.type === 'function_call_output' && item.message?.name === 'request_mission_plan'
    );
    const toolResultForLLM = realToolResult?.message ?? {
      type: 'function_call_output',
      name: 'request_mission_plan',
      output: newOutput,
    };

    this.continueAfterTools(
      [toolResultForLLM],
      chat_id,
      sessionId,
      systemInstructions,
      allowedTools,
      false,
      agentProfile,
      true // skipPersist: new row already inserted at step 3
    ).catch((err) => {
      chatLogger.error(`[returnMissionPlanXYZ] continueAfterTools failed for chat ${chat_id}:`, err);
      eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, {
        chatId: chat_id,
        from: 'assistant',
        timestamp: new Date().toISOString(),
        message: {
          role: 'assistant',
          content: `Error al procesar el plan de misión: ${err.message}`,
          type: 'text',
          status: 'error',
        },
      });
    });

    return { ok: true, msg: 'Mission plan received. Main chat processing resumed.' };
  }
}
