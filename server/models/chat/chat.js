import { MCPclient } from './mcpClient.js';
import { LLMFactory } from './llmFactory.js';
import { chatLogger } from '../../common/logger.js';
import { LLM, MCPenable } from '../../config/config.js';
import { SystemPrompts } from './prompts/index.js';
import { eventBus, EVENTS } from '../../common/eventBus.js';
import { ChatPersistenceModel } from './chatPersistence.js';
import { convertMissionBriefingToXYZ, convertMissionXYZToLatLong } from './coordinateConverter.js';
import { missionController } from '../../controllers/mission.js';
/**
 * Orquestador que maneja el flujo completo de procesamiento de mensajes
 * entre el LLM y el servidor MCP
 * elemnt of conversation
 *  {
 *   id: 'unique-message-id',
 *   timestamp: '2024-01-01T12:00:00Z',
 *   chatId: 'chat-1234',
 *   from: 'user' | 'assistant',
 *   provider: 'openai' | 'anthropic' | 'gemini',
 *   message: {
 *    role: 'user' | 'assistant' | 'system' | 'tool',}
 *    content: 'message content' | {tool call or tool result}
 *    type: 'text' | 'tool_call' | 'tool_result',
 *    status: 'pending' | 'completed' | 'error',
 *    summary: 'optional summary of the message',
 *    call_id: 'if applicable, the id of the tool call',
 *    input: 'if applicable, the input to the tool call',
 *    output: 'if applicable, the output from the tool call',
 *    tool:
 *    name: 'tool name',
 *     ...
 *    }
 * }
 */

let conversationHistories = {}; // Map of chatId -> conversation history
let mcpClient = null;
let llmHandler = null;
const maxIterations = 6; // Prevenir loops infinitos

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
    return llmHandler !== null && llmHandler !== undefined;
  }

  /**
   * Procesa un mensaje completo, manejando llamadas a herramientas si es necesario
   * @param {string} chatId - ID de la conversación
   * @param {string} message - Mensaje del usuario
   * @returns {Promise<Object>} Respuesta final
   */
  static async processMessage(chatId, message) {
    chatLogger.info('═════════════════════════════════════════');
    chatLogger.info(`📨 Chat: ${chatId} | Mensaje: "${message}"`);
    chatLogger.info('═══════════════════════════════════════════');

    // Get or create conversation history for this chatId
    if (!conversationHistories[chatId]) {
      chatLogger.info(`🆕 Creando nuevo historial de conversación para chat: ${chatId}`);

      // Try to load from database
      try {
        const dbHistory = await ChatPersistenceModel.loadHistory(chatId);
        conversationHistories[chatId] = dbHistory;
        chatLogger.info(`📂 Loaded ${dbHistory.length} messages from DB for chat: ${chatId}`);
      } catch (error) {
        chatLogger.error('Error loading history from DB:', error);
        conversationHistories[chatId] = [];
      }
    }
    const conversationHistory = conversationHistories[chatId];

    try {
      // Obtener herramientas según el proveedor de LLM
      const tools = this.getToolsForProvider();
      // add system prompt if history is empty
      if (conversationHistory.length === 0 && SystemPrompts.main) {
        const mySystemPrompt = `${SystemPrompts.main}\n\n---\nSession context:\n- chat_id: ${chatId}`;

        await ChatPersistenceModel.addMessageToChat(
          chatId,
          'system',
          { role: 'system', content: mySystemPrompt },
          conversationHistory
        );
      }

      // Agregar el mensaje del usuario al historial
      await ChatPersistenceModel.addMessageToChat(
        chatId,
        'user',
        { role: 'user', content: message },
        conversationHistory
      );

      let response = await llmHandler.processMessage(message, tools, conversationHistory.slice(0, -1));

      let toolCallsFlag = false;
      for (const res of response) {
        chatLogger.debug('Assistant message:', JSON.stringify(res));

        if (res.type === 'function_call' || res.type === 'tool_call') {
          toolCallsFlag = true;
        }
        await ChatPersistenceModel.addMessageToChat(chatId, 'assistant', res, conversationHistory);
      }
      // chatLogger.debug('History:', JSON.stringify(conversationHistory, null, 2));

      if (toolCallsFlag) {
        this.handleToolCallsLoop(response, tools, chatId);
      }

      chatLogger.info('✓ Procesamiento completado para chat:', chatId);

      return llmHandler.normalizeResponse(response);
    } catch (error) {
      chatLogger.error('Error procesando mensaje:', error);
      eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, {
        chatId,
        message: {
          role: 'assistant',
          content: `Error: ${error.message}`,
          status: 'error',
        },
        timestamp: new Date().toISOString(),
      });

      throw error;
    }
  }

  /**
   * Maneja el loop de llamadas a herramientas
   */
  static async handleToolCallsLoop(response, _tools, chatId) {
    let isToolCalling = true;
    let iterations = 0;
    chatLogger.debug('Starting tool calls loop...');
    chatLogger.debug('Initial response with tool calls:', JSON.stringify(response, null, 2));
    let currentResponse = response;

    while (isToolCalling && iterations < maxIterations) {
      iterations++;
      chatLogger.debug(`Iteration ${iterations} - Processing tools...`);
      isToolCalling = true;
      const toolResults = await this.executeToolCalls(currentResponse);
      currentResponse = await this.continueAfterTools(toolResults, chatId);

      isToolCalling = currentResponse.some(
        (content) => content.type === 'function_call' || content.type === 'tool_call'
      );
    }
    chatLogger.debug('Tool calls loop finished.');

    if (iterations >= maxIterations) {
      chatLogger.warn('Maximum iterations reached');
    }

    return currentResponse;
  }

  /**
   * Ejecuta todas las llamadas a herramientas solicitadas por el LLM
   */
  static async executeToolCalls(toolCalls) {
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
   */
  static async continueAfterTools(toolResults, chatId) {
    const conversationHistory = conversationHistories[chatId];
    for (const res of toolResults) {
      await ChatPersistenceModel.addMessageToChat(chatId, 'assistant', res, conversationHistory);
    }

    const chatresponse = await llmHandler.processMessage(null, this.getToolsForProvider(), conversationHistory);

    chatLogger.debug('Response after tool execution:', JSON.stringify(chatresponse, null, 2));

    for (const res of chatresponse) {
      await ChatPersistenceModel.addMessageToChat(chatId, 'assistant', res, conversationHistory);
    }
    return chatresponse;
  }

  /**
   * Obtiene las herramientas en el formato correcto para el proveedor actual
   */
  static getToolsForProvider() {
    if (!mcpClient || !mcpClient.isReady()) {
      chatLogger.debug('No MCP tools available');
      return [];
    }

    const tools = mcpClient.getTools();

    if (!tools || tools.length === 0) {
      chatLogger.debug('No MCP tools available');
      return [];
    }
    return tools;
  }

  /**
   * Obtiene el historial de conversación para un chat específico
   * @param {string} chatId - ID del chat
   */
  static async getHistory(chatId) {
    // If not in memory, try to load from DB
    if (!conversationHistories[chatId]) {
      try {
        const dbHistory = await ChatPersistenceModel.loadHistory(chatId);
        conversationHistories[chatId] = dbHistory;
      } catch (error) {
        chatLogger.error('Error loading history from DB:', error);
      }
    }
    return conversationHistories[chatId] || [];
  }

  /**
   * Lista todos los chats disponibles
   */
  static async listChats() {
    const now = new Date().toISOString();
    const inMemoryChats = Object.keys(conversationHistories).map((chatId) => ({
      id: chatId,
      messageCount: conversationHistories[chatId].length,
      lastUpdated: now,
      createdAt: now,
      source: 'memory',
    }));

    try {
      const dbChats = await ChatPersistenceModel.getAllChats();
      const chatMap = new Map(inMemoryChats.map((c) => [c.id, c]));

      for (const dbChat of dbChats) {
        if (!chatMap.has(dbChat.id)) {
          const messageCount = await ChatPersistenceModel.getMessageCount(dbChat.id);
          chatMap.set(dbChat.id, {
            id: dbChat.id,
            name: dbChat.name,
            messageCount: messageCount,
            lastUpdated: dbChat.updatedAt,
            createdAt: dbChat.createdAt,
            source: 'database',
          });
        }
      }

      return Array.from(chatMap.values());
    } catch (error) {
      chatLogger.error('Error listing DB chats:', error);
      return inMemoryChats;
    }
  }

  /**
   * Elimina un chat completamente
   * @param {string} chatId - ID del chat a eliminar
   * @param {boolean} hardDelete - Si true, elimina permanentemente de la BD
   */
  static async deleteChat(chatId, hardDelete = false) {
    // Remove from memory
    delete conversationHistories[chatId];

    // Remove from database
    try {
      await ChatPersistenceModel.deleteChat(chatId, hardDelete);
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
      await ChatPersistenceModel.updateChat(chatId, { name });
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
      const chat = await ChatPersistenceModel.createChat(name);
      // Initialize empty conversation history in memory
      conversationHistories[chat.id] = [];
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
    chatLogger.info(
      `[MainChat: ${mainChatId}] Coordinates converted. Origin: lat=${origin_global.lat}, lng=${origin_global.lng}, alt=${origin_global.alt}`
    );

    // ═══════════════════════════════════════════════════════════════════
    // STEP 2: Create secondary chat for background mission planning
    // ═══════════════════════════════════════════════════════════════════
    const secondaryChat = await this.createChat(`MP-XYZ-${mainChatId}`);
    const secondaryChatId = secondaryChat.id;
    chatLogger.info(`[MainChat: ${mainChatId}] Secondary chat created: ${secondaryChatId}`);

    // Initialize conversation history for secondary chat
    if (!conversationHistories[secondaryChatId]) {
      conversationHistories[secondaryChatId] = [];
    }
    const secondaryChatHistory = conversationHistories[secondaryChatId];

    // ═══════════════════════════════════════════════════════════════════
    // STEP 3: Configure secondary chat with mission-specific system prompt
    // ═══════════════════════════════════════════════════════════════════
    const systemPromptContent = `${SystemPrompts.mission_build_xyz}

---
Session context:
- main_chat_id: ${mainChatId}
- secondary_chat_id: ${secondaryChatId}
- coordinate_system: local XYZ (ENU - East/North/Up in meters)
- origin: lat=${origin_global.lat}, lng=${origin_global.lng}, alt=${origin_global.alt}`;

    await ChatPersistenceModel.addMessageToChat(
      secondaryChatId,
      'system',
      { role: 'system', content: systemPromptContent },
      secondaryChatHistory
    );

    // ═══════════════════════════════════════════════════════════════════
    // STEP 4: Build the mission planning request message
    // ═══════════════════════════════════════════════════════════════════
    const userMessage = `Create a mission plan based on the following data in local XYZ coordinates (meters):

## Origin (Global Reference)
- Latitude: ${origin_global.lat}
- Longitude: ${origin_global.lng}
- Altitude: ${origin_global.alt}m

## Devices Information
${JSON.stringify(missionDataXYZ.drone_information, null, 2)}

## Elements to Inspect
${JSON.stringify(missionDataXYZ.target_elements, null, 2)}

## group Information
${JSON.stringify(missionDataXYZ.group_information, null, 2)}

## Mission Requirements
${JSON.stringify(missionDataXYZ.mission_requirements, null, 2)}

## User Context
${JSON.stringify(missionDataXYZ.user_context, null, 2)}

Generate an optimized mission plan to cover all inspection elements using the available devices. Provide waypoints in local XYZ coordinates.`;

    // ═══════════════════════════════════════════════════════════════════
    // STEP 5: Start background processing (non-blocking)
    // ═══════════════════════════════════════════════════════════════════
    // processMessage handles LLM interaction and tool calls asynchronously
    this.processMessage(secondaryChatId, userMessage);

    chatLogger.info(`[MainChat: ${mainChatId}] Background processing started in secondary chat: ${secondaryChatId}`);

    // Return immediately - secondary chat continues processing in background
    return { secondaryChatId, msg: 'Mission is processing.' };
  }

  static async generateMissionBriefing(missionData) {
    chatLogger.info(`[MissionBriefing] Starting mission briefing generation`);

    const mission = convertMissionXYZToLatLong(missionData);
    const response = await missionController.showMission(mission);

    chatLogger.info(`[MissionBriefing] Mission briefing generation completed`);

    return { msg: 'Mission briefing created successfully.', response };
  }
}
