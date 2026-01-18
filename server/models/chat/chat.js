import { MCPclient } from './mcpClient.js';
import { LLMFactory } from './llmFactory.js';
import { chatLogger } from '../../common/logger.js';
import { LLM, MCPenable } from '../../config/config.js';
import { SystemPrompts } from './SystemPromps.js';
import { eventBus, EVENTS } from '../../common/eventBus.js';
import { ChatPersistenceModel } from './chatPersistence.js';
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
      if (conversationHistory.length === 0 && SystemPrompts.openai) {
        await ChatPersistenceModel.addMessageToChat(
          chatId,
          'system',
          { role: 'system', content: SystemPrompts.openai },
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
    const inMemoryChats = Object.keys(conversationHistories).map((chatId) => ({
      id: chatId,
      messageCount: conversationHistories[chatId].length,
      lastUpdated: new Date().toISOString(),
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
}
