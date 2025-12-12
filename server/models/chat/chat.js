import { MCPclient } from './mcpClient.js';
import { LLMFactory } from './llmFactory.js';
import logger, { chatLogger } from '../../common/logger.js';
import { LLM, LLMType, LLMApiKey, MCPenable, MCPconfig } from '../../config/config.js';
import { SystemPrompts } from './SystemPromps.js';
import { eventBus, EVENTS } from '../../common/eventBus.js';
import { time } from 'console';
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

var conversationHistories = {}; // Map of chatId -> conversation history
var mcpClient = null;
var llmHandler = null;
var maxIterations = 6; // Prevenir loops infinitos

export function initializeLLMProvider(provider, apiKey) {}

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
      conversationHistories[chatId] = [];
    }
    const conversationHistory = conversationHistories[chatId];

    try {
      // Obtener herramientas según el proveedor de LLM
      const tools = this.getToolsForProvider();

      if (conversationHistory.length === 0 && SystemPrompts.openai) {
        conversationHistory.push({
          chatId,
          from: 'system',
          timestamp: new Date().toISOString(),
          message: { role: 'system', content: SystemPrompts.openai },
        });
      }

      // Agregar el mensaje del usuario al historial
      conversationHistory.push({
        chatId,
        from: 'user',
        timestamp: new Date().toISOString(),
        message: { role: 'user', content: message },
      });

      let response = await llmHandler.processMessage(message, tools, conversationHistory.slice(0, -1));

      let toolCallsFlag = false;
      response.map((res) => {
        console.log('Assistant message :', JSON.stringify(res));

        if (res.type === 'function_call' || res.type === 'tool_call') {
          toolCallsFlag = true;
        }
        const chatItem = {
          chatId,
          from: 'assistant',
          timestamp: new Date().toISOString(),
          message: res,
        };
        eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, chatItem);
        conversationHistory.push(chatItem);
      });
      console.log('history:', JSON.stringify(conversationHistory, null, 2));

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
  static async handleToolCallsLoop(response, tools, chatId) {
    let isToolCalling = true;
    let iterations = 0;
    console.log('\n🔄 Iniciando loop de llamadas a herramientas...\n');
    console.log('Respuesta inicial con llamadas a herramientas:', JSON.stringify(response, null, 2));
    let currentResponse = response;

    while (isToolCalling && iterations < maxIterations) {
      iterations++;
      console.log(`\n🔄 Iteración ${iterations} - Procesando herramientas...\n`);
      isToolCalling = true;
      const toolResults = await this.executeToolCalls(currentResponse);
      currentResponse = await this.continueAfterTools(toolResults, chatId);

      isToolCalling = currentResponse.some((content) => {
        if (content.type === 'function_call' || content.type === 'tool_call') {
          return true;
        }
        return false;
      });
    }
    console.log('\n🔄 Loop de llamadas a herramientas finalizado.\n');

    if (iterations >= maxIterations) {
      console.warn('⚠️  Se alcanzó el máximo de iteraciones');
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
    //conversationHistory.push(...toolResults);
    toolResults.map((res) => {
      const chatItem = {
        chatId,
        from: 'assistant',
        timestamp: new Date().toISOString(),
        message: res,
      };
      eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, chatItem);
      conversationHistory.push(chatItem);
    });

    const chatresponse = await llmHandler.processMessage(null, this.getToolsForProvider(), conversationHistory);

    console.log(
      `Respuesta después de procesar la respuesta de la funcion:`,
      JSON.stringify(chatresponse, null, 2)
    );

    chatresponse.map((res) => {
      const chatItem = {
        chatId,
        from: 'assistant',
        timestamp: new Date().toISOString(),
        message: res,
      };
      eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, chatItem);
      conversationHistory.push(chatItem);
    });
    return chatresponse;
  }

  /**
   * Obtiene las herramientas en el formato correcto para el proveedor actual
   */
  static getToolsForProvider() {
    if (!mcpClient.isReady()) {
      console.log('ℹ️  No hay herramientas MCP disponibles');
      return [];
    }

    const tools = mcpClient.getTools();

    if (!tools || tools.length === 0) {
      console.log('ℹ️  No hay herramientas MCP disponibles');
      return [];
    }
    return tools;
  }

  /**
   * Reinicia el historial de conversación
   * @param {string} chatId - ID del chat a reiniciar (opcional, si no se proporciona reinicia todos)
   */
  static resetHistory(chatId = null) {
    if (chatId) {
      // Reset specific chat
      conversationHistories[chatId] = [];
      console.log(`🔄 Historial de conversación reiniciado para chat: ${chatId}`);
    } else {
      // Reset all chats
      conversationHistories = {};
      console.log('🔄 Todos los historiales de conversación reiniciados');
    }
  }

  /**
   * Obtiene el historial de conversación para un chat específico
   * @param {string} chatId - ID del chat
   */
  static getHistory(chatId) {
    return conversationHistories[chatId] || [];
  }

  /**
   * Lista todos los chats disponibles
   */
  static listChats() {
    return Object.keys(conversationHistories).map((chatId) => ({
      id: chatId,
      messageCount: conversationHistories[chatId].length,
      lastUpdated: new Date().toISOString(),
    }));
  }
}
