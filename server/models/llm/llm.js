import { MCPManager } from '../../mcp/mcpManager.js';
import { llmHandlerFactory } from './llmHandlerFactory.js';

/**
 * Orquestador que maneja el flujo completo de procesamiento de mensajes
 * entre el LLM y el servidor MCP
 */

var history = [];
var mcpManager = new MCPManager();
var llmHandler = llmHandlerFactory.createHandler();
var maxIterations = 5; // Prevenir loops infinitos

export class MessageOrchestrator {
  /**
   * Procesa un mensaje completo, manejando llamadas a herramientas si es necesario
   * @param {string} message - Mensaje del usuario
   * @returns {Promise<Object>} Respuesta final
   */
  async processMessage(message) {
    console.log('\n═══════════════════════════════════════════════════');
    console.log(`📨 Nuevo mensaje: "${message}"`);
    console.log('═══════════════════════════════════════════════════\n');

    try {
      // Obtener herramientas según el proveedor de LLM
      const tools = this.getToolsForProvider();

      // Agregar el mensaje del usuario al historial
      this.conversationHistory.push({
        role: 'user',
        content: message,
      });

      let response = await this.llmHandler.processMessage(
        message,
        tools,
        this.conversationHistory.slice(0, -1) // Excluir el último que acabamos de agregar
      );

      // Si el LLM quiere usar herramientas, procesarlas
      if (response.type === 'tool_calls') {
        response = await this.handleToolCallsLoop(response, tools);
      }

      // Agregar la respuesta final al historial
      if (response.message) {
        this.conversationHistory.push(response.message);
      }

      console.log('\n═══════════════════════════════════════════════════');
      console.log('✓ Procesamiento completado');
      console.log('═══════════════════════════════════════════════════\n');

      return this.llmHandler.normalizeResponse(response);
    } catch (error) {
      console.error('Error procesando mensaje:', error);
      throw error;
    }
  }

  /**
   * Maneja el loop de llamadas a herramientas
   */
  async handleToolCallsLoop(response, tools) {
    let currentResponse = response;
    let iterations = 0;

    while (currentResponse.type === 'tool_calls' && iterations < this.maxIterations) {
      iterations++;
      console.log(`\n🔄 Iteración ${iterations} - Procesando herramientas...\n`);

      const toolResults = await this.executeToolCalls(currentResponse.toolCalls);

      // Continuar la conversación con los resultados de las herramientas
      currentResponse = await this.continueAfterTools(currentResponse, toolResults);
    }

    if (iterations >= this.maxIterations) {
      console.warn('⚠️  Se alcanzó el máximo de iteraciones');
    }

    return currentResponse;
  }

  /**
   * Ejecuta todas las llamadas a herramientas solicitadas por el LLM
   */
  async executeToolCalls(toolCalls) {
    const results = [];

    for (const toolCall of toolCalls) {
      const result = await this.llmHandler.handleToolCall(toolCall, async (name, args) => {
        return await this.mcpClient.executeTool(name, args);
      });
      results.push(result);
    }

    return results;
  }

  /**
   * Continúa la conversación después de ejecutar herramientas
   */
  async continueAfterTools(response, toolResults) {
    const provider = this.llmHandler.getProviderName();

    if (provider === 'openai') {
      // Para OpenAI: agregar el mensaje del asistente y los resultados de tools
      this.conversationHistory.push(response.message);
      this.conversationHistory.push(...toolResults);

      return await this.llmHandler.continueWithToolResults(this.conversationHistory, toolResults);
    } else if (provider === 'anthropic') {
      // Para Anthropic: manejar el formato específico
      return await this.llmHandler.continueWithToolResults(
        this.conversationHistory,
        response.response.content,
        toolResults
      );
    }

    throw new Error(`Proveedor no soportado: ${provider}`);
  }

  /**
   * Obtiene las herramientas en el formato correcto para el proveedor actual
   */
  getToolsForProvider() {
    if (!this.mcpClient.isReady()) {
      console.log('ℹ️  No hay herramientas MCP disponibles');
      return [];
    }

    const provider = this.llmHandler.getProviderName();

    if (provider === 'openai') {
      return this.mcpClient.getToolsForOpenAI();
    } else if (provider === 'anthropic') {
      return this.mcpClient.getToolsForAnthropic();
    }

    return [];
  }

  /**
   * Reinicia el historial de conversación
   */
  resetHistory() {
    this.conversationHistory = [];
    console.log('🔄 Historial de conversación reiniciado');
  }

  /**
   * Obtiene el historial de conversación
   */
  getHistory() {
    return this.conversationHistory;
  }
}
