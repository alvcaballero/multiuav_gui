import OpenAI from 'openai';
import { BaseLLMHandler } from './baseLLMhandler.js';
import { SystemPrompts } from './prompts/index.js';
import { logger, chatLogger } from '../../common/logger.js';

//suported roles= 'assistant', 'system', 'developer', and 'user'
// models: gpt-4.1, gpt-4o, o4-mini, gpt-5, gpt-5-mini,, gpt-5-nano gpt-5.2,etc.
class OpenAIHandler extends BaseLLMHandler {
  constructor(apiKey, model = 'gpt-5.2', systemPrompt = SystemPrompts.openai) {
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
    chatLogger.info(`✓ OpenAI client initialized (model: ${this.model})`);
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

  _parseAssistantResponse(assistantMessage) {
    let msgType = 'text';
    let responseMsg = {
      role: 'assistant',
      content: '',
    };
    let toolCalls = [];

    for (const content of assistantMessage) {
      if (content.type === 'reasoning') {
        chatLogger.info('Reasoning:', JSON.stringify(content.summary || content, null, 2).substring(0, 30) + '...');
      }
      if (content.type === 'function_call' || content.type === 'tool_call') {
        chatLogger.info(`✓ Tool call request: ${content.name}`);
        msgType = 'tool_calls';
        toolCalls.push(content);
        responseMsg.role = 'tool';
        responseMsg.name = `execute_${content.name}`;
      }
      if (content.type === 'text' || content.type === 'message') {
        chatLogger.info(`✓ Partial response: ${(content.text || content.content?.text || '').substring(0, 30)}...`);
        responseMsg.role = 'assistant';
        responseMsg.content += content.text || content.content?.text || '';
        msgType = 'text';
        if (Array.isArray(content.content)) {
          for (const block of content.content) {
            if (block.type === 'output_text') {
              chatLogger.info(`✓ Output text block: ${block.text.substring(0, 30)}...`);
              responseMsg.content = responseMsg.content === '' ? block.text : '';
            }
          }
        }
      }
    }

    return { msgType, responseMsg, toolCalls };
  }

  async processMessage(message = null, tools = [], conversationHistory = []) {
    if (!this.client) {
      throw new Error('OpenAI client not initialized');
    }

    // Build the messages array
    const messages = this.convertMsg(message, conversationHistory);

    //console.log('Messages to send to OpenAI:', JSON.stringify(messages, null, 2));
    // Parameters for the call
    const params = {
      model: this.model,
      input: messages,
      reasoning: { effort: 'high' },
    };

    // Add tools if available
    if (tools.length > 0) {
      params.tools = this.convertToolsForMCP(tools);
      params.tool_choice = 'auto';
    }

    try {
      chatLogger.info(`→ Sending message to OpenAI...`);
      const response = await this.client.responses.create(params);
      const assistantMessage = response.output;

      // Parse response (result unused here, but parsing logs the response)
      this._parseAssistantResponse(assistantMessage);

      return assistantMessage;
    } catch (error) {
      chatLogger.error('Error in OpenAI:', error);
      throw error;
    }
  }

  async handleToolCall(toolCall, toolExecutor) {
    chatLogger.info('Handling tool call:', JSON.stringify(toolCall, null, 2).substring(0, 30) + '...');
    try {
      const functionName = toolCall.name;
      const functionArgs = JSON.parse(toolCall.arguments);

      // Execute the tool through MCP
      const result = await toolExecutor(functionName, functionArgs);
      chatLogger.info(`✓ Tool ${functionName} response:`, JSON.stringify(result, null, 2).substring(0, 30) + '...');

      // Response format for OpenAI
      return {
        type: 'function_call_output',
        call_id: toolCall.call_id,
        output: JSON.stringify(result),
      };
    } catch (error) {
      chatLogger.error('Error handling tool call:', error);
      return {
        type: 'function_call_output',
        call_id: toolCall.call_id,
        output: JSON.stringify({ error: error.message }),
      };
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
