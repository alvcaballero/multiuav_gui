import { MCPclient } from './mcpClient.js';
import { LLMFactory } from './llmFactory.js';
import logger,{chatLogger} from '../../common/logger.js';
import { LLM, LLMType, LLMApiKey, MCPenable, MCPconfig } from '../../config/config.js';

/**
 * Orquestador que maneja el flujo completo de procesamiento de mensajes
 * entre el LLM y el servidor MCP
 */
var conversationHistory = [];
var mcpClient = null;
var llmHandler = null;
var maxIterations = 5; // Prevenir loops infinitos

export function initializeLLMProvider(provider, apiKey) {}

export class MessageOrchestrator {
  static initializeLLMProvider(provider,apiKey) {
    if (LLM && !llmHandler) {
      llmHandler = LLMFactory.createHandler(provider, apiKey);
      llmHandler.initialize().then(() => {
        chatLogger.info('LLM Handler initialized successfully.');
      }).catch((error) => {
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
   * @param {string} message - Mensaje del usuario
   * @returns {Promise<Object>} Respuesta final
   */
  static async processMessage(message) {
    chatLogger.info('═════════════════════════════════════════');
    chatLogger.info(`📨 Nuevo mensaje: "${message}"`);
    chatLogger.info('═══════════════════════════════════════════');

    try {
      // Obtener herramientas según el proveedor de LLM
      const tools = this.getToolsForProvider();

      // Agregar el mensaje del usuario al historial
      conversationHistory.push({
        role: 'user',
        content: message,
      });

      let response = await llmHandler.processMessage(
        message,
        tools,
        conversationHistory.slice(0, -1) // Excluir el último que acabamos de agregar
      );
      console.log('Respuesta inicial del LLM:', JSON.stringify(response, null, 2));

      // Si el LLM quiere usar herramientas, procesarlas
      if (response.type === 'tool_calls') {
        response = await this.handleToolCallsLoop(response, tools);
      }

      // Agregar la respuesta final al historial
      if (response.message) {
        conversationHistory.push(response.message);
      }

      console.log('\n═══════════════════════════════════════════════════');
      console.log('✓ Procesamiento completado');
      console.log('═══════════════════════════════════════════════════\n');

      return llmHandler.normalizeResponse(response);
    } catch (error) {
      console.error('Error procesando mensaje:', error);
      throw error;
    }
  }

  /**
   * Maneja el loop de llamadas a herramientas
   */
  static async handleToolCallsLoop(response, tools) {
    let currentResponse = response;
    let iterations = 0;
    console.log('\n🔄 Iniciando loop de llamadas a herramientas...\n');
    console.log('Respuesta inicial con llamadas a herramientas:', JSON.stringify(currentResponse, null, 2));

    while (currentResponse.type === 'tool_calls' && iterations < maxIterations) {
      iterations++;
      console.log(`\n🔄 Iteración ${iterations} - Procesando herramientas...\n`);

      const toolResults = await this.executeToolCalls(currentResponse.toolCalls);

      // Continuar la conversación con los resultados de las herramientas
      currentResponse = await this.continueAfterTools(currentResponse, toolResults);
      console.log(`Respuesta después de procesar herramientas en la iteración ${iterations}:`, JSON.stringify(currentResponse, null, 2));
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
      const result = await llmHandler.handleToolCall(toolCall, async (name, args) => {
        return await mcpClient.executeTool(name, args);
      });
      results.push(result);
    }

    return results;
  }

  /**
   * Continúa la conversación después de ejecutar herramientas
   */
  static async continueAfterTools(response, toolResults) {
    const provider = llmHandler.getProviderName();

    if (provider === 'openai') {
      // Para OpenAI: agregar el mensaje del asistente y los resultados de tools
      conversationHistory.push(response.message);
      conversationHistory.push(...toolResults);

      return await llmHandler.continueWithToolResults(conversationHistory, toolResults);
    } else if (provider === 'anthropic') {
      // Para Anthropic: manejar el formato específico
      return await llmHandler.continueWithToolResults(
        conversationHistory,
        response.response.content,
        toolResults
      );
    }

    throw new Error(`Proveedor no soportado: ${provider}`);
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
    
    const provider = llmHandler.getProviderName();

    if (provider === 'openai') {
      return mcpClient.getToolsForOpenAI();
    } else if (provider === 'anthropic') {
      return mcpClient.getToolsForAnthropic();
    } else if (provider === 'gemini') {
      return mcpClient.getToolsForGemini();
    }

    return [];
  }

  /**
   * Reinicia el historial de conversación
   */
  resetHistory() {
    conversationHistory = [];
    console.log('🔄 Historial de conversación reiniciado');
  }

  /**
   * Obtiene el historial de conversación
   */
  getHistory() {
    return conversationHistory;
  }
}
