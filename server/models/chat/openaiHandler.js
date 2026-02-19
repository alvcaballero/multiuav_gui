import OpenAI from 'openai';
import { BaseLLMHandler } from './baseLLMhandler.js';
import { SystemPrompts } from './prompts/index.js';
import { logger, chatLogger } from '../../common/logger.js';

//suported roles= 'assistant', 'system', 'developer', and 'user'
// models: gpt-4.1, gpt-4o, o4-mini, gpt-5, gpt-5-mini,, gpt-5-nano gpt-5.2,etc.
class OpenAIHandler extends BaseLLMHandler {
  static AGENT_PROFILES = {
    default: {
      model: 'gpt-5-mini-2025-08-07',
      reasoning: { effort: 'low' },
    },
    planner: {
      model: 'gpt-5-2025-08-07',
      reasoning: { effort: 'medium' },
    },
  };

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
    this.initialized = true;
    chatLogger.info(`✓ OpenAI client initialized (model: ${this.model})`);
  }

  /**
   * Ensures an OpenAI conversation exists for the given chat.
   * Creates one if it doesn't exist yet.
   * @param {string} chatId - Internal chat identifier
   * @param {Object} persistence - Adapter with { getSessionId, setSessionId, clearSession }
   * @returns {Promise<string|null>} OpenAI conversation ID or null on failure
   */
  async ensureSession(chatId, persistence) {
    let sessionId = await persistence.getSessionId(chatId);
    if (sessionId) return sessionId;

    try {
      sessionId = await this._createConversation({ chatId });
      await persistence.setSessionId(chatId, sessionId);
      chatLogger.info(`Created OpenAI conversation for chat ${chatId}: ${sessionId}`);
      return sessionId;
    } catch (error) {
      chatLogger.error('Failed to create OpenAI conversation, will use full history:', error);
      return null;
    }
  }

  /**
   * Handles session-related errors from processMessage.
   * Clears invalid sessions and creates replacements.
   * @param {string} chatId - Internal chat identifier
   * @param {Error} error - The error from processMessage
   * @param {Object} persistence - Adapter with { getSessionId, setSessionId, clearSession }
   * @returns {Promise<boolean>} true if session was recovered (caller should clear sessionId for retry)
   */
  async handleSessionError(chatId, error, persistence) {
    if (this._isConversationLockedError(error)) {
      await persistence.clearSession(chatId);
      chatLogger.warn(`Conversation locked for chat ${chatId}, cleared for next message`);
      return false; // Not recoverable within this request
    }

    if (error.sessionCleared) {
      // processMessage already fell back to full history, now recreate session for next message
      await persistence.clearSession(chatId);
      try {
        const newSessionId = await this._createConversation({ chatId });
        await persistence.setSessionId(chatId, newSessionId);
        chatLogger.info(`Created replacement OpenAI conversation for chat ${chatId}: ${newSessionId}`);
      } catch (recreateError) {
        chatLogger.error('Failed to create replacement conversation:', recreateError);
      }
      return true; // The result is already good (fallback succeeded)
    }

    return false;
  }

  /**
   * Creates a new OpenAI conversation (internal helper).
   * @param {Object} metadata - Optional metadata
   * @returns {Promise<string>} The conversation ID
   * @private
   */
  async _createConversation(metadata = {}) {
    if (!this.client) {
      throw new Error('OpenAI client not initialized');
    }
    const conversation = await this.client.conversations.create({ metadata });
    chatLogger.info(`✓ OpenAI conversation created: ${conversation.id}`);
    return conversation.id;
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
      sessionId = null, // Provider session ID (e.g., OpenAI conversation ID)
      previousResponseId = null, // DEPRECATED: Response chaining (kept for backwards compatibility)
      instructions = null,
      toolOutputs = null,
      allowedTools = null, // list of allowed tool names for this call
      forceFinish = false, // force final response with system message
      agentProfile = 'default', // Agent profile for model/reasoning selection
    } = options;

    // Resolve agent profile for model and reasoning config
    const profile = this.getAgentProfile(agentProfile);

    // Message to force LLM to provide final response when tool limit is reached
    const forceFinishMessage = {
      role: 'system',
      content:
        'Maximum tool iterations reached. You MUST provide your final response NOW using only the information gathered so far. Do NOT attempt to call any more tools. Summarize what was accomplished and present the results to the user.',
    };

    // Parameters for the call — model and reasoning come from the agent profile
    const params = {
      model: profile.model || this.model,
      reasoning: profile.reasoning || { effort: 'medium' },
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

    // CASE 1: Using sessionId (Conversations API - preferred, persistent 30 days)
    if (sessionId) {
      params.conversation = sessionId;

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

    chatLogger.info('tools')
    for (const tool of tools) {
      chatLogger.info(`✓ ${tool.name}: ${tool.description.substring(0, 100)}...`);
    }
    chatLogger.info(`✓ Message for OpenAI`);
    for (const msg of messages) {
      chatLogger.info(`- role: ${msg.role}, content: ${typeof msg.content === 'string' ? msg.content.replace(/\r?\n|\r/g, " ").substring(0, 100) + '...' : JSON.stringify(msg.content).replace(/\r?\n|\r/g, " ").substring(0, 100) + '...'}`);
    }
    try {
      const logId = sessionId
        ? `sessionId: ${sessionId.substring(0, 20)}...`
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

      // If conversation_locked error (concurrent access), signal to continue without conversation
      if (this._isConversationLockedError(error)) {
        chatLogger.warn(`Conversation locked error, will continue without persistent conversation`);
        const wrappedError = new Error(error.message);
        wrappedError.code = 'conversation_locked';
        wrappedError.recoverable = true;
        wrappedError.userMessage =
          'La conversación está siendo procesada por otra solicitud. Puedes continuar chateando, pero el historial de esta sesión no se mantendrá en el servidor.';
        throw wrappedError;
      }

      // If sessionId fails (expired, not found, or missing tool outputs), try to recreate or fall back
      if (sessionId && (this._isConversationError(error) || this._isResponseIdError(error))) {
        chatLogger.warn(`sessionId failed (${error.message}), falling back to full history`);
        const result = await this.processMessage(message, tools, conversationHistory, {
          sessionId: null,
          previousResponseId: null,
          instructions,
        });
        result.sessionCleared = true; // Signal for handleSessionError to recreate
        return result;
      }

      // If previous_response_id fails (expired, not found, missing tool outputs), fall back to full history
      if (previousResponseId && this._isResponseIdError(error)) {
        chatLogger.warn(`previous_response_id failed (${error.message}), falling back to full history`);
        const result = await this.processMessage(message, tools, conversationHistory, {
          sessionId: null,
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

  /**
   * Check if the error is a conversation_locked error (concurrent access)
   * @param {Error} error - The error object
   * @returns {boolean} True if it's a conversation_locked error
   */
  _isConversationLockedError(error) {
    return error.code === 'conversation_locked' || error.message?.toLowerCase().includes('conversation_locked');
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
        name: functionName,
        output: JSON.stringify(result),
      };
    } catch (error) {
      chatLogger.error('Error handling tool call:', error);
      return {
        type: 'function_call_output',
        call_id: toolCall.call_id,
        name: toolCall.name,
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
