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

  /**
   * Creates a new conversation in OpenAI for persistent state management.
   * Conversations persist for 30 days and enable automatic prompt caching.
   * @param {Object} metadata - Optional metadata to attach to the conversation
   * @returns {Promise<string>} The conversation ID
   */
  async createConversation(metadata = {}) {
    if (!this.client) {
      throw new Error('OpenAI client not initialized');
    }
    try {
      const conversation = await this.client.conversations.create({
        metadata,
      });
      chatLogger.info(`✓ OpenAI conversation created: ${conversation.id}`);
      return conversation.id;
    } catch (error) {
      chatLogger.error('Error creating OpenAI conversation:', error);
      throw error;
    }
  }

  /**
   * Retrieves an existing conversation from OpenAI.
   * @param {string} conversationId - The conversation ID to retrieve
   * @returns {Promise<Object>} The conversation object
   */
  async getConversation(conversationId) {
    if (!this.client) {
      throw new Error('OpenAI client not initialized');
    }
    try {
      const conversation = await this.client.conversations.retrieve(conversationId);
      return conversation;
    } catch (error) {
      chatLogger.error(`Error retrieving conversation ${conversationId}:`, error);
      throw error;
    }
  }

  /**
   * Checks if a conversation exists and is valid.
   * @param {string} conversationId - The conversation ID to check
   * @returns {Promise<boolean>} True if the conversation exists
   */
  async isConversationValid(conversationId) {
    if (!conversationId) return false;
    try {
      await this.getConversation(conversationId);
      return true;
    } catch (error) {
      return false;
    }
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

  async processMessage(message = null, tools = [], conversationHistory = [], options = {}) {
    if (!this.client) {
      throw new Error('OpenAI client not initialized');
    }

    const {
      conversationId = null, // NEW: OpenAI Conversations API
      previousResponseId = null, // DEPRECATED: Response chaining (kept for backwards compatibility)
      instructions = null,
      toolOutputs = null,
      allowedTools = null, // NEW: list of allowed tool names for this call
      forceFinish = false, // NEW: force final response with system message
    } = options;

    // Message to force LLM to provide final response when tool limit is reached
    const forceFinishMessage = {
      role: 'system',
      content:
        'Maximum tool iterations reached. You MUST provide your final response NOW using only the information gathered so far. Do NOT attempt to call any more tools. Summarize what was accomplished and present the results to the user.',
    };

    // Parameters for the call
    const params = {
      model: this.model,
      reasoning: { effort: 'high' },
    };

    // Add tools if available
    // If allowedTools is empty array, don't include tools (forces text-only response)
    if (tools.length > 0 && (!allowedTools || allowedTools.length > 0)) {
      params.tools = this.convertToolsForMCP(tools);
      params.tool_choice = 'auto';
      if (allowedTools && allowedTools.length > 0) {
        params.tool_choice = {
          type: 'allowed_tools',
          mode: 'auto',
          tools: allowedTools.map((toolName) => ({ type: 'function', name: toolName })),
        };
      }
    }

    // CASE 1: Using conversationId (Conversations API - preferred, persistent 30 days)
    if (conversationId) {
      params.conversation = conversationId;

      // Tool continuation - must send tool outputs in input
      if (toolOutputs && toolOutputs.length > 0) {
        params.input = [...toolOutputs];
        // Add force finish message after tool outputs to ensure final response
        if (forceFinish) {
          params.input.push(forceFinishMessage);
        }
      }
      // New user message
      else if (message !== null) {
        params.input = [{ role: 'user', content: message }];
      }
      // Empty continuation (shouldn't happen normally)
      else {
        params.input = [];
      }

      // Instructions can be sent on first message of conversation
      if (instructions) {
        params.instructions = instructions;
      }
    }
    // CASE 2: Using previous_response_id (legacy response chaining - backwards compatibility)
    else if (previousResponseId) {
      params.previous_response_id = previousResponseId;

      // Tool continuation - must send tool outputs in input
      if (toolOutputs && toolOutputs.length > 0) {
        params.input = [...toolOutputs];
        // Add force finish message after tool outputs to ensure final response
        if (forceFinish) {
          params.input.push(forceFinishMessage);
        }
      }
      // New user message
      else if (message !== null) {
        params.input = [{ role: 'user', content: message }];
      }
      // Empty continuation (shouldn't happen normally)
      else {
        params.input = [];
      }

      // Re-send instructions if provided (they don't persist in chains)
      if (instructions) {
        params.instructions = instructions;
      }
    }
    // CASE 3: First message or fallback (full history path)
    else {
      params.input = this.convertMsg(message, conversationHistory);
    }

    try {
      const logId = conversationId
        ? `conversationId: ${conversationId.substring(0, 20)}...`
        : previousResponseId
          ? `previousResponseId: ${previousResponseId.substring(0, 20)}...`
          : 'none';
      chatLogger.info(`→ Sending message to OpenAI (${logId})...`);
      const response = await this.client.responses.create(params);

      // Verificar el estado de la respuesta
      if (response.status !== 'completed') {
        chatLogger.error(
          `OpenAI response status: ${response.status} | ID: ${response.id || 'N/A'} | Model: ${response.model || this.model}`
        );
        if (response.status === 'failed' && response.error) {
          chatLogger.error(`Error: [${response.error.code || 'unknown'}] ${response.error.message || 'No message'}`);
        }
        if (response.status === 'incomplete' && response.incomplete_details) {
          chatLogger.error(`Incomplete reason: ${response.incomplete_details.reason || 'unknown'}`);
        }
      }

      const assistantMessage = response.output;

      // Parse response (result unused here, but parsing logs the response)
      this._parseAssistantResponse(assistantMessage);

      // Return extended result with response metadata
      return {
        output: assistantMessage,
        responseId: response.id,
        model: response.model || this.model,
        status: response.status,
      };
    } catch (error) {
      chatLogger.error('Error in OpenAI:', error);

      // If conversationId fails (expired, not found, or missing tool outputs), try to recreate or fall back
      if (conversationId && (this._isConversationError(error) || this._isResponseIdError(error))) {
        chatLogger.warn(`conversationId failed (${error.message}), falling back to full history`);
        const result = await this.processMessage(message, tools, conversationHistory, {
          conversationId: null,
          previousResponseId: null,
          instructions,
        });
        result.conversationIdCleared = true; // Signal to create new conversation
        return result;
      }

      // If previous_response_id fails (expired, not found, missing tool outputs), fall back to full history
      if (previousResponseId && this._isResponseIdError(error)) {
        chatLogger.warn(`previous_response_id failed (${error.message}), falling back to full history`);
        const result = await this.processMessage(message, tools, conversationHistory, {
          conversationId: null,
          previousResponseId: null,
          instructions,
        });
        result.responseIdCleared = true; // Signal to clear stored responseId
        return result;
      }
      throw error;
    }
  }

  /**
   * Check if the error is related to an invalid/expired response ID or missing tool outputs
   * @param {Error} error - The error object
   * @returns {boolean} True if it's a response ID error that should trigger fallback
   */
  _isResponseIdError(error) {
    const message = error.message?.toLowerCase() || '';
    return (
      (message.includes('response') &&
        (message.includes('not found') || message.includes('expired') || message.includes('invalid'))) ||
      message.includes('tool output') ||
      message.includes('function call')
    );
  }

  /**
   * Check if the error is related to an invalid/expired conversation
   * @param {Error} error - The error object
   * @returns {boolean} True if it's a conversation error that should trigger fallback
   */
  _isConversationError(error) {
    const message = error.message?.toLowerCase() || '';
    return (
      (message.includes('conversation') &&
        (message.includes('not found') || message.includes('expired') || message.includes('invalid'))) ||
      message.includes('conv_')
    );
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
    // Handle both old format (array) and new format (object with output)
    const output = Array.isArray(response) ? response : response.output;
    const responseId = response.responseId || null;

    return {
      provider: 'openai',
      content: output,
      model: response.model || this.model,
      responseId: responseId,
      raw: response,
    };
  }

  getProviderName() {
    return 'openai';
  }
}
export { OpenAIHandler };
