import { Ollama } from 'ollama';
import { BaseLLMHandler } from './baseLLMhandler.js';
import { SystemPrompts } from './prompts/index.js';
import { chatLogger } from '../../common/logger.js';


// llama-px4  ,glm-4.7-flash,llama3.1:8b, etc.
class OllamaHandler extends BaseLLMHandler {
  static AGENT_PROFILES = {
    default: {
      model: 'glm-4.7-flash',
    },
    planner: {
      model: 'glm-4.7-flash',
    },
  };

  constructor(apiKey, model = 'glm-4.7-flash', systemPrompt = SystemPrompts.main) {
    super(apiKey, model, systemPrompt);
    if (!apiKey) {
      throw new Error('Ollama host URL is required for OllamaHandler.');
    }
  }

  async initialize() {
    // Custom fetch with extended timeout (8 min) to avoid HeadersTimeoutError
    // on slow LLM inference with large contexts and tool calling
    const OLLAMA_TIMEOUT_MS = 8 * 60 * 1000;

    const fetchWithTimeout = (url, options = {}) => {
      const timeoutSignal = AbortSignal.timeout(OLLAMA_TIMEOUT_MS);
      const signal = options.signal
        ? AbortSignal.any([options.signal, timeoutSignal])
        : timeoutSignal;
      return fetch(url, { ...options, signal });
    };

    this.client = new Ollama({ host: this.apiKey, fetch: fetchWithTimeout });
    this.initialized = true;
    chatLogger.info(`✓ Ollama client initialized (host: ${this.apiKey}, model: ${this.model}, timeout: ${OLLAMA_TIMEOUT_MS / 1000}s)`);
  }

  /**
   * Converts MCP tools to Ollama format (identical to OpenAI).
   */
  convertToolsForMCP(tools) {
    return tools.map((tool) => ({
      type: 'function',
      function: {
        name: tool.name,
        description: tool.description,
        parameters: tool.inputSchema,
      },
    }));
  }

  /**
   * Converts conversation history to Ollama's messages format.
   * Ollama supports system, user, assistant, and tool roles inline.
   */
  convertMsg(message = null, conversationHistory) {
    const messages = [];

    for (const msg of conversationHistory) {
      const item = msg.message || msg;
      const role = item.role;
      const type = item.type;

      // System messages passed inline
      if (role === 'system') {
        messages.push({ role: 'system', content: item.content });
        continue;
      }

      // function_call → assistant with tool_calls
      if (type === 'function_call') {
        let args = {};
        try {
          args = typeof item.arguments === 'string' ? JSON.parse(item.arguments) : (item.arguments || {});
        } catch { /* keep empty */ }
        messages.push({
          role: 'assistant',
          content: '',
          tool_calls: [{
            function: {
              name: item.name,
              arguments: args,
            },
          }],
        });
        continue;
      }

      // function_call_output → tool role
      if (type === 'function_call_output') {
        const output = typeof item.output === 'string' ? item.output : JSON.stringify(item.output || {});
        messages.push({
          role: 'tool',
          content: output,
        });
        continue;
      }

      // Text content
      const content = item.content;
      if (typeof content === 'string') {
        messages.push({ role: role === 'assistant' ? 'assistant' : 'user', content });
      }
    }

    if (message !== null) {
      messages.push({ role: 'user', content: message });
    }

    return messages;
  }

  /**
   * Parses Ollama chat response into normalized output array.
   */
  _parseOllamaResponse(response) {
    const output = [];
    const msg = response.message;

    chatLogger.info('Parsing Ollama response...');
    chatLogger.info(`✓ Full response: ${JSON.stringify(response).substring(0, 1000000)}...`);

    if (msg.tool_calls && msg.tool_calls.length > 0) {
      for (const tc of msg.tool_calls) {
        chatLogger.info(`✓ Tool call request: ${tc.function.name}`);
        output.push({
          type: 'function_call',
          name: tc.function.name,
          arguments: JSON.stringify(tc.function.arguments || {}),
          call_id: `ollama_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`,
        });
      }
    }

    if (msg.content && msg.content.length > 0) {
      chatLogger.info(`✓ Response: ${msg.content.substring(0, 30)}...`);
      output.push({
        type: 'text',
        content: msg.content,
        role: 'assistant',
      });
    }

    return output;
  }

  /**
   * Processes a message following the same interface as the other handlers.
   */
  async processMessage(message = null, tools = [], conversationHistory = [], options = {}) {
    if (!this.client) {
      throw new Error('Ollama client not initialized');
    }

    const {
      instructions = null,
      toolOutputs = null,
      allowedTools = null,
      forceFinish = false,
      agentProfile = 'default',
    } = options;

    const profile = this.getAgentProfile(agentProfile);
    const modelId = profile.model || this.model;

    // Build messages from conversation history
    let messages;

    if (message !== null) {
      messages = this.convertMsg(message, conversationHistory);
    } else {
      messages = this.convertMsg(null, conversationHistory);
    }

    if (toolOutputs && toolOutputs.length > 0) {
      // Append tool outputs as tool role messages
      for (const output of toolOutputs) {
        const content = typeof output.output === 'string' ? output.output : JSON.stringify(output.output || {});
        messages.push({
          role: 'tool',
          content,
        });
      }
      if (forceFinish) {
        messages.push({
          role: 'system',
          content: 'Maximum tool iterations reached. You MUST provide your final response NOW using only the information gathered so far. Do NOT attempt to call any more tools.',
        });
      }
    }
    // Prepend system prompt only if not already present in conversation history
    const systemText = instructions || this.systemPrompt;
    const hasSystemMessage = messages.some((m) => m.role === 'system');
    if (systemText && !hasSystemMessage) {
      messages.unshift({ role: 'system', content: systemText });
    }

    // Build tools config
    const ollamaTools = (tools.length > 0 && (!allowedTools || allowedTools.length > 0))
      ? this.convertToolsForMCP(tools)
      : undefined;

    chatLogger.info('tools')
    for (const tool of tools) {
      chatLogger.info(`✓ ${tool.name}: ${tool.description.substring(0, 100)}...`);
    }
    chatLogger.info(`✓ Message for Ollama`);
    for (const msg of messages) {
      chatLogger.info(`- role: ${msg.role}, content: ${typeof msg.content === 'string' ? msg.content.replace(/\r?\n|\r/g, " ").substring(0, 100) + '...' : JSON.stringify(msg.content).replace(/\r?\n|\r/g, " ").substring(0, 100) + '...'}`);
    }
    try {
      chatLogger.info(`→ Sending message to Ollama (model: ${modelId})...`);

      const response = await this.client.chat({
        model: modelId,
        messages,
        tools: ollamaTools,
        stream: false,
        think: false,
        options: {
          num_ctx: 16384,
          temperature: 0.2,
          
        }
      });

      const output = this._parseOllamaResponse(response);

      return {
        output,
        responseId: null,
        model: modelId,
        status: 'completed',
      };
    } catch (error) {
      chatLogger.error('Error in Ollama:', error);
      throw error;
    }
  }

  /**
   * Handles a tool call, matching the canonical output format.
   */
  async handleToolCall(toolCall, toolExecutor) {
    chatLogger.info('Handling tool call:', JSON.stringify(toolCall, null, 2).substring(0, 30) + '...');
    try {
      const functionName = toolCall.name;
      const functionArgs = JSON.parse(toolCall.arguments);

      const result = await toolExecutor(functionName, functionArgs);
      chatLogger.info(`✓ Tool ${functionName} response:`, JSON.stringify(result, null, 2).substring(0, 30) + '...');

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
    const output = Array.isArray(response) ? response : response.output;

    return {
      provider: 'ollama',
      content: output,
      model: response.model || this.model,
      responseId: null,
      raw: response,
    };
  }

  getProviderName() {
    return 'ollama';
  }
}

export { OllamaHandler };
