import Anthropic from '@anthropic-ai/sdk';
import { BaseLLMHandler } from './baseLLMhandler.js';
import { SystemPrompts } from './prompts/index.js';
import { chatLogger } from '../../common/logger.js';

// Models: claude-opus-4-6, claude-haiku-4-5-20251001, claude-sonnet-4-5-20250929, etc.
class AnthropicHandler extends BaseLLMHandler {
  static AGENT_PROFILES = {
    default: {
      model: 'claude-haiku-4-5-20251001',
    },
    planner: {
      model: 'claude-sonnet-4-5-20250929',
      maxTokens: 8192,
    },
  };

  constructor(apiKey, model = 'claude-haiku-4-5-20251001', systemPrompt = SystemPrompts.main) {
    super(apiKey, model, systemPrompt);
    if (!apiKey) {
      throw new Error('Anthropic API Key is required for AnthropicHandler.');
    }
  }

  async initialize() {
    this.client = new Anthropic({ apiKey: this.apiKey });
    this.initialized = true;
    chatLogger.info(`✓ Anthropic client initialized (model: ${this.model})`);
  }

  /**
   * Converts MCP tools to Anthropic's tool format.
   * Anthropic uses input_schema (JSON Schema) directly.
   */
  convertToolsForMCP(tools) {
    return tools.map((tool) => ({
      name: tool.name,
      description: tool.description,
      input_schema: tool.inputSchema,
    }));
  }

  /**
   * Converts conversation history to Anthropic messages format.
   * Anthropic requires alternating user/assistant messages.
   * System messages are handled via the 'system' parameter.
   */
  convertMsg(message = null, conversationHistory) {
    const messages = [];

    for (const msg of conversationHistory) {
      const item = msg.message || msg;
      const role = item.role;
      const type = item.type;

      // Skip system messages — handled via system parameter
      if (role === 'system') continue;

      // Map tool role to user with tool_result content
      if (role === 'tool') continue;

      // Normalized function_call from DB → Anthropic tool_use block (assistant role)
      if (type === 'function_call') {
        let input = {};
        try {
          input = typeof item.arguments === 'string' ? JSON.parse(item.arguments) : (item.arguments || {});
        } catch { /* keep empty */ }
        messages.push({
          role: 'assistant',
          content: [{
            type: 'tool_use',
            id: item.call_id,
            name: item.name,
            input,
          }],
        });
        continue;
      }

      // Normalized function_call_output from DB → Anthropic tool_result block (user role)
      if (type === 'function_call_output') {
        messages.push({
          role: 'user',
          content: [{
            type: 'tool_result',
            tool_use_id: item.call_id,
            content: item.output || '',
          }],
        });
        continue;
      }

      // Text content (normalized or legacy)
      const content = item.content;
      const anthropicRole = role === 'assistant' ? 'assistant' : 'user';

      if (typeof content === 'string') {
        messages.push({ role: anthropicRole, content });
      } else if (Array.isArray(content)) {
        messages.push({ role: anthropicRole, content });
      }
    }

    if (message !== null) {
      messages.push({ role: 'user', content: message });
    }

    return messages;
  }

  /**
   * Parses Anthropic response into the normalized output array format
   * that the orchestrator expects (matching OpenAI's output structure).
   */
  _parseAnthropicResponse(response) {
    const output = [];

    for (const block of response.content) {
      if (block.type === 'tool_use') {
        chatLogger.info(`✓ Tool call request: ${block.name}`);
        output.push({
          type: 'function_call',
          name: block.name,
          arguments: JSON.stringify(block.input || {}),
          call_id: block.id,
        });
      } else if (block.type === 'text') {
        chatLogger.info(`✓ Response: ${block.text.substring(0, 30)}...`);
        output.push({
          type: 'text',
          content: block.text,
          role: 'assistant',
        });
      }
    }

    return output;
  }

  /**
   * Processes a message following the same interface as OpenAIHandler.
   * Returns { output: Array, responseId: string|null, model: string, status: string }
   */
  async processMessage(message = null, tools = [], conversationHistory = [], options = {}) {
    if (!this.client) {
      throw new Error('Anthropic client not initialized');
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
    const maxTokens = profile.maxTokens || 4096;

    // Build params
    const params = {
      model: modelId,
      max_tokens: maxTokens,
    };

    // System prompt (Anthropic uses a top-level 'system' param, not in messages)
    const systemText = instructions || this.systemPrompt;
    if (systemText) {
      params.system = systemText;
    }

    // Add tools if available
    if (tools.length > 0 && (!allowedTools || allowedTools.length > 0)) {
      params.tools = this.convertToolsForMCP(tools);
    }

    // Build messages
    if (toolOutputs && toolOutputs.length > 0) {
      // Tool continuation: build history + tool results
      const messages = this.convertMsg(null, conversationHistory);

      // The last assistant message should contain the tool_use blocks.
      // Anthropic requires: assistant message with tool_use → user message with tool_result
      const toolResultContent = toolOutputs.map((output) => ({
        type: 'tool_result',
        tool_use_id: output.call_id,
        content: output.output,
      }));

      if (forceFinish) {
        toolResultContent.push({
          type: 'text',
          text: 'Maximum tool iterations reached. You MUST provide your final response NOW using only the information gathered so far. Do NOT attempt to call any more tools.',
        });
      }

      messages.push({ role: 'user', content: toolResultContent });
      params.messages = messages;
    } else if (message !== null) {
      params.messages = this.convertMsg(message, conversationHistory);
    } else {
      params.messages = this.convertMsg(null, conversationHistory);
    }

    // Ensure we have at least one message
    if (!params.messages || params.messages.length === 0) {
      params.messages = [{ role: 'user', content: 'Continue.' }];
    }

    try {
      chatLogger.info(`→ Sending message to Anthropic (model: ${modelId})...`);

      const response = await this.client.messages.create(params);

      // Check stop reason
      if (response.stop_reason === 'end_turn' || response.stop_reason === 'tool_use') {
        chatLogger.info(`Anthropic response stop_reason: ${response.stop_reason}`);
      } else if (response.stop_reason === 'max_tokens') {
        chatLogger.warn(`Anthropic response truncated (max_tokens reached)`);
      }

      const output = this._parseAnthropicResponse(response);

      return {
        output,
        responseId: response.id || null,
        model: response.model || modelId,
        status: 'completed',
      };
    } catch (error) {
      chatLogger.error('Error in Anthropic:', error);
      throw error;
    }
  }

  /**
   * Handles a tool call from Anthropic, matching the format
   * that processMessage expects as toolOutputs.
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
      provider: 'anthropic',
      content: output,
      model: response.model || this.model,
      responseId: response.responseId || null,
      raw: response,
    };
  }

  getProviderName() {
    return 'anthropic';
  }
}

export { AnthropicHandler };
