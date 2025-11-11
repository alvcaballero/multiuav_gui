import Anthropic from '@anthropic-ai/sdk';
import { BaseLLMHandler } from './baseLLMHandler.js';

/**
 * Manejador para Anthropic Claude models
 */
export class AnthropicHandler extends BaseLLMHandler {
  constructor(apiKey, model = 'claude-3-5-sonnet-20241022') {
    super(apiKey, model);
  }

  async initialize() {
    this.client = new Anthropic({
      apiKey: this.apiKey,
    });
    console.log(`✓ Cliente Anthropic inicializado (modelo: ${this.model})`);
  }

  async processMessage(message, tools = [], conversationHistory = []) {
    if (!this.client) {
      throw new Error('Cliente Anthropic no inicializado');
    }

    // Convertir historial al formato de Anthropic
    const messages = this.convertHistoryToAnthropicFormat(conversationHistory);
    messages.push({ role: 'user', content: message });

    // Parámetros para la llamada
    const params = {
      model: this.model,
      max_tokens: 4096,
      messages: messages,
    };

    // Agregar tools si están disponibles
    if (tools.length > 0) {
      params.tools = tools;
    }

    try {
      console.log(`→ Enviando mensaje a Anthropic...`);
      const response = await this.client.messages.create(params);

      // Verificar si hay tool calls
      const toolUseBlocks = response.content.filter(block => block.type === 'tool_use');
      
      if (toolUseBlocks.length > 0) {
        console.log(`✓ Anthropic solicita ${toolUseBlocks.length} llamada(s) a herramientas`);
        return {
          type: 'tool_calls',
          response: response,
          toolCalls: toolUseBlocks,
        };
      }

      // Extraer texto de la respuesta
      const textBlock = response.content.find(block => block.type === 'text');
      const content = textBlock ? textBlock.text : '';

      console.log(`✓ Respuesta recibida de Anthropic`);
      return {
        type: 'text',
        content: content,
        response: response,
      };
    } catch (error) {
      console.error('Error en Anthropic:', error);
      throw error;
    }
  }

  async handleToolCall(toolCall, toolExecutor) {
    try {
      const functionName = toolCall.name;
      const functionArgs = toolCall.input;

      // Ejecutar la herramienta a través del MCP
      const result = await toolExecutor(functionName, functionArgs);

      // Formato de respuesta para Anthropic
      return {
        type: 'tool_result',
        tool_use_id: toolCall.id,
        content: JSON.stringify(result),
      };
    } catch (error) {
      console.error('Error manejando tool call:', error);
      return {
        type: 'tool_result',
        tool_use_id: toolCall.id,
        content: JSON.stringify({ error: error.message }),
        is_error: true,
      };
    }
  }

  /**
   * Continúa la conversación después de ejecutar herramientas
   */
  async continueWithToolResults(conversationHistory, assistantContent, toolResults) {
    // Agregar el mensaje del asistente con los tool_use
    conversationHistory.push({
      role: 'assistant',
      content: assistantContent,
    });

    // Agregar los resultados de las herramientas
    conversationHistory.push({
      role: 'user',
      content: toolResults,
    });

    const params = {
      model: this.model,
      max_tokens: 4096,
      messages: conversationHistory,
    };

    try {
      console.log(`→ Continuando conversación con resultados de herramientas...`);
      const response = await this.client.messages.create(params);
      
      const textBlock = response.content.find(block => block.type === 'text');
      const content = textBlock ? textBlock.text : '';

      return {
        type: 'text',
        content: content,
        response: response,
      };
    } catch (error) {
      console.error('Error continuando conversación:', error);
      throw error;
    }
  }

  /**
   * Convierte el historial de conversación al formato de Anthropic
   */
  convertHistoryToAnthropicFormat(history) {
    return history.map(msg => {
      if (msg.role === 'assistant' && msg.tool_calls) {
        // Convertir tool_calls de OpenAI a formato Anthropic
        return {
          role: 'assistant',
          content: msg.tool_calls.map(tc => ({
            type: 'tool_use',
            id: tc.id,
            name: tc.function.name,
            input: JSON.parse(tc.function.arguments),
          })),
        };
      }
      return msg;
    });
  }

  normalizeResponse(response) {
    return {
      provider: 'anthropic',
      content: response.content,
      model: this.model,
      raw: response,
    };
  }

  getProviderName() {
    return 'anthropic';
  }
}