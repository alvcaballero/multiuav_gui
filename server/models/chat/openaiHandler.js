import OpenAI from 'openai';
import { BaseLLMHandler } from './baseLLMhandler.js';
import { SystemPrompts } from './prompts/index.js';
import { logger, chatLogger } from '../../common/logger.js';

//suported roles= 'assistant', 'system', 'developer', and 'user'
class OpenAIHandler extends BaseLLMHandler {
  constructor(apiKey, model = 'gpt-5', systemPrompt = SystemPrompts.openai) {
    //logger.info(`Apikey in handler ${apiKey}, ${model}`);
    super(apiKey, model, systemPrompt);
    if (!apiKey) {
      throw new Error('OpenAI API Key is required for OpenAIHandler.');
    }
  }

  async initialize() {
    this.client = new OpenAI({
      apiKey: this.apiKey,
    });
    chatLogger.info(`✓ Cliente OpenAI inicializado (modelo: ${this.model})`);
  }

  convertToolsForMCP(tools) {
    return tools.map((tool) => {
      return {
        type: 'function',
        name: tool.name,
        description: tool.description,
        parameters: tool.inputSchema,
      };
    });
  }

  convertMsg(message = null, conversationHistory) {
    let lastconversation = conversationHistory.map((msg) => {
      if (typeof msg.message.content === 'string') {
        return {
          role: msg.message.role,
          content: msg.message.content,
        };
      } else {
        return { ...msg.message };
      }
    });
    if (message === null) {
      return lastconversation;
    }
    return [...lastconversation, { role: 'user', content: message }];
  }

  async processMessage(message = null, tools = [], conversationHistory = []) {
    if (!this.client) {
      throw new Error('Cliente OpenAI no inicializado');
    }

    // Construir el array de mensajes
    const messages = this.convertMsg(message, conversationHistory);

    console.log('Messages to send to OpenAI:', JSON.stringify(messages, null, 2));
    // Parámetros para la llamada
    const params = {
      model: this.model,
      input: messages,
    };

    // Agregar tools si están disponibles
    if (tools.length > 0) {
      params.tools = this.convertToolsForMCP(tools);
      params.tool_choice = 'auto';
    }

    try {
      chatLogger.info(`→ Enviando mensaje a OpenAI...`);
      //const response = await this.client.chat.completions.create(params);
      const response = await this.client.responses.create(params);
      //const assistantMessage = response.choices[0].message;

      chatLogger.info('OpenAI Response:', JSON.stringify(response, null, 2));
      const assistantMessage = response.output;
      console.log('Assistant message from OpenAI:', JSON.stringify(assistantMessage, null, 2));
      let msgType = 'text';
      let responseMsg = {
        role: 'assistant',
        content: '',
      };
      let toolCalls = [];
      for (const content of assistantMessage) {
        console.log('Processing content from assistantMessage:', JSON.stringify(content, null, 2));
        if (content.type === 'reasoning') {
          chatLogger.info('Reasoning:', JSON.stringify(content, null, 2));
        }
        if (content.type === 'function_call' || content.type === 'tool_call') {
          chatLogger.info(`✓ OpenAI solicita llamada a herramienta: ${content.name}`);
          msgType = 'tool_calls';
          toolCalls.push(content);
          responseMsg.role = 'tool';
          responseMsg.name = `execute_${content.name}`;
        }
        if (content.type === 'text' || content.type === 'message') {
          chatLogger.info(`✓ Respuesta parcial de OpenAI: ${content.text || content.content?.text}`);
          responseMsg.role = 'assistant';
          responseMsg.content += content.text || content.content?.text || '';
          msgType = 'text';
          if (Array.isArray(content.content)) {
            for (const block of content.content) {
              if (block.type === 'output_text') {
                chatLogger.info(`✓ Bloque de texto de salida: ${block.text}`);
                responseMsg.content = responseMsg.content === '' ? block.text : '';
              }
            }
          }
        }
      }
      return assistantMessage;
    } catch (error) {
      console.error('Error en OpenAI:', error);
      throw error;
    }
  }

  async handleToolCall(toolCall, toolExecutor) {
    console.log('Manejando tool call:', JSON.stringify(toolCall, null, 2));
    try {
      const functionName = toolCall.name;
      const functionArgs = JSON.parse(toolCall.arguments);

      // Ejecutar la herramienta a través del MCP
      const result = await toolExecutor(functionName, functionArgs);
      console.log(`✓ Herramienta  ejecutada con éxito ${functionName}. Resultado:`, result);

      // Formato de respuesta para OpenAI
      return {
        type: 'function_call_output',
        call_id: toolCall.call_id,
        output: JSON.stringify(result),
      };
    } catch (error) {
      console.error('Error manejando tool call:', error);
      return {
        type: 'function_call_output',
        call_id: toolCall.call_id,
        output: JSON.stringify({ error: error.message }),
      };
    }
  }

  /**
   * Continúa la conversación después de ejecutar herramientas
   */
  async continueWithToolResults(conversationHistory, toolResults) {
    const params = {
      model: this.model,
      input: conversationHistory,
    };

    try {
      console.log(`→ Continuando conversación con resultados de herramientas...`);
      console.log('Conversation history:', JSON.stringify(conversationHistory, null, 2));
      //const response = await this.client.chat.completions.create(params);
      const response = await this.client.responses.create(params);
      const assistantMessage = response.output;
      console.log('Assistant message from OpenAI:', JSON.stringify(assistantMessage, null, 2));
      let msgType = 'text';
      let responseMsg = {
        role: 'assistant',
        content: '',
      };
      let toolCalls = [];
      for (const content of assistantMessage) {
        if (content.type === 'reasoning') {
          chatLogger.info('Reasoning:', JSON.stringify(content.summary, null, 2));
        }
        if (content.type === 'function_call' || content.type === 'tool_call') {
          chatLogger.info(`✓ OpenAI solicita llamada a herramienta: ${content.name}`);
          msgType = 'tool_calls';
          toolCalls.push(content);
          responseMsg.role = 'tool';
          responseMsg.name = `execute_${content.name}`;
        }
        if (content.type === 'text' || content.type === 'message') {
          chatLogger.info(`✓ Respuesta parcial de OpenAI: ${content.text || content.content?.text}`);
          responseMsg.role = 'assistant';
          responseMsg.content += content.text || content.content?.text || '';
          msgType = 'text';
          if (Array.isArray(content.content)) {
            for (const block of content.content) {
              if (block.type === 'output_text') {
                chatLogger.info(`✓ Bloque de texto de salida: ${block.text}`);
                responseMsg.content = responseMsg.content === '' ? block.text : '';
              }
            }
          }
        }
      }

      // Verificar si hay tool calls
      if (msgType === 'tool_calls') {
        console.log(`✓ OpenAI solicita ${toolCalls.length} llamada(s) a herramientas`);
        return {
          type: 'tool_calls',
          content: assistantMessage,
          message: responseMsg,
          toolCalls: toolCalls,
        };
      }

      console.log(`✓ Respuesta recibida de OpenAI`);
      return {
        type: 'text',
        content: assistantMessage,
        message: responseMsg,
      };
    } catch (error) {
      console.error('Error continuando conversación:', error);
      throw error;
    }
  }

  normalizeResponse(response) {
    return {
      provider: 'openai',
      content: response.content,
      model: this.model,
      raw: response,
      ...response.message,
    };
  }

  getProviderName() {
    return 'openai';
  }
}
export { OpenAIHandler };
