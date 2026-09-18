import OpenAI from 'openai';
import { BaseLLMHandler, makeUsage, renderSubagentResult } from './baseLLMhandler.js';
import { SystemPrompts } from '../agents/index.js';
import { chatLogger } from '../../../common/logger.js';

/**
 * Handler for any server that speaks the OpenAI **Chat Completions** wire protocol
 * (`POST {baseURL}/chat/completions`): Ollama, llama.cpp (`llama-server`), vLLM,
 * LM Studio, OpenRouter, Groq…
 *
 * ── Why this is NOT OpenAIHandler ──────────────────────────────────────────────
 * `OpenAIHandler` targets the **Responses API** (`client.responses.create`), which
 * gives it server-side conversations, `reasoning` items and `tool_choice:
 * {type:'allowed_tools'}`. No third-party server implements that API. Collapsing
 * the two would mean amputating those features from OpenAI proper, so the two
 * transports stay apart and only this one is shared.
 *
 * ── The capability rule (do not break it) ──────────────────────────────────────
 * `capabilities` may only decide **what we SEND** (request shaping). It must never
 * decide **how we PARSE**. Parsing is unconditional and defensive, because a server
 * can stop honouring a declared capability without us changing a line — see
 * llama.cpp#20198, where `arguments` turned into a parsed object and broke every
 * client that trusted the spec. An `if (capabilities.x) parseA() else parseB()` in
 * here is a subclass hiding inside an object: it destroys the "zero new code per
 * provider" property this class exists for.
 */
class OpenAICompatibleHandler extends BaseLLMHandler {
  /**
   * Local servers serve one model at a time, so every tier maps to the same id.
   * A deployment with several models overrides this via `capabilities.capabilityMap`.
   */
  static CAPABILITY_MAP = {
    low: {},
    medium: {},
    high: {},
  };

  /**
   * @param {string} apiKey - Token for the endpoint. Local servers ignore it, but the
   *   OpenAI SDK refuses to construct without one, hence the 'not-needed' placeholder.
   * @param {string} model - Model id to request.
   * @param {string} systemPrompt
   * @param {object} opts
   * @param {string} opts.baseURL - REQUIRED. Full base URL including the `/v1` suffix.
   * @param {string} [opts.providerName] - Reported by getProviderName(); persisted in usage rows.
   * @param {object} [opts.capabilities] - Request-shaping only. See the class note.
   */
  constructor(apiKey, model, systemPrompt = SystemPrompts.main, opts = {}) {
    const { baseURL, providerName = 'openai-compatible', capabilities = {} } = opts;
    super(apiKey || 'not-needed', model, systemPrompt);
    if (!baseURL) {
      throw new Error(`baseURL is required for OpenAICompatibleHandler (provider: ${providerName}).`);
    }
    if (!model) {
      throw new Error(`model is required for OpenAICompatibleHandler (provider: ${providerName}).`);
    }
    this.baseURL = baseURL;
    this.providerName = providerName;
    this.capabilities = {
      /** Sent as `tool_choice`. Ollama's /v1 does NOT support the field — leave false there. */
      toolChoice: false,
      /** llama.cpp disables multiple tool calls unless asked; OpenAI defaults to true. null = omit. */
      parallelToolCalls: null,
      /** Maps to `reasoning_effort`. 'none' is how Ollama's /v1 expresses "don't think". */
      reasoningEffort: null,
      /** Maps to `max_tokens`. null = let the server decide. */
      maxTokens: null,
      temperature: 0.2,
      /** Verbatim extra body fields for servers with non-standard knobs. */
      extraBody: {},
      /** Logged once at initialize(): a deployment requirement this protocol cannot enforce. */
      startupWarning: null,
      ...capabilities,
    };
    if (this.capabilities.capabilityMap) {
      this.capabilityMap = this.capabilities.capabilityMap;
    }
  }

  /**
   * Tier lookup falls back to the instance-level map when a deployment supplied one,
   * since the static CAPABILITY_MAP is shared by every compatible provider.
   */
  resolveModelConfig(agent) {
    if (this.capabilityMap) {
      const tier = agent?.capability ?? 'low';
      return this.capabilityMap[tier] || this.capabilityMap['low'] || { model: this.model };
    }
    return { model: this.model };
  }

  async initialize() {
    // Local inference on a large context with tool calling routinely blows past the
    // SDK's default timeout, so it is raised to match the old native Ollama client.
    const TIMEOUT_MS = 8 * 60 * 1000;
    this.client = new OpenAI({
      apiKey: this.apiKey,
      baseURL: this.baseURL,
      timeout: TIMEOUT_MS,
      maxRetries: 2,
    });
    this.initialized = true;
    chatLogger.info(
      `✓ OpenAI-compatible client initialized (provider: ${this.providerName}, baseURL: ${this.baseURL}, ` +
        `model: ${this.model}, timeout: ${TIMEOUT_MS / 1000}s)`
    );
    if (this.capabilities.startupWarning) {
      chatLogger.warn(`[${this.providerName}] ${this.capabilities.startupWarning}`);
    }
  }

  /** Chat Completions nests the schema under `function`, unlike the Responses API. */
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
   * Chat Completions requires `arguments` to be a JSON **string**, while our canonical
   * item may legitimately hold either shape (llama.cpp hands back parsed objects).
   * @param {*} args
   * @returns {string}
   */
  _stringifyArguments(args) {
    if (typeof args === 'string') return args;
    try {
      return JSON.stringify(args ?? {});
    } catch {
      return '{}';
    }
  }

  /**
   * Inverse of the above, applied to whatever the server returned.
   * Unconditional on purpose — see the capability rule in the class note.
   * @param {*} args
   * @returns {string} Canonical `arguments` (always a JSON string)
   */
  _normalizeArguments(args) {
    if (typeof args === 'string') return args;
    return this._stringifyArguments(args);
  }

  /** Servers that omit tool-call ids still need a stable id to pair call ↔ result. */
  _synthesizeCallId() {
    return `${this.providerName}_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`;
  }

  /**
   * Rebuilds the Chat Completions message list from persisted canonical items.
   *
   * Tool pairing is the delicate part: an `assistant` message carrying `tool_calls`
   * MUST be followed by one `tool` message per call, each echoing the matching
   * `tool_call_id`. The old native-Ollama path omitted those ids (its `/api/chat`
   * tolerated it); a spec-compliant endpoint does not, and a Jinja chat template
   * renders the pairing directly, so a missing id corrupts the prompt.
   */
  convertHistory(conversationHistory) {
    const messages = [];

    for (const msg of conversationHistory) {
      const item = msg.message || msg;
      const { role, type } = item;

      if (role === 'system') {
        messages.push({ role: 'system', content: item.content });
        continue;
      }

      if (type === 'function_call') {
        const callId = item.call_id || this._synthesizeCallId();
        messages.push({
          role: 'assistant',
          content: '',
          tool_calls: [
            {
              id: callId,
              type: 'function',
              function: { name: item.name, arguments: this._stringifyArguments(item.arguments) },
            },
          ],
        });
        continue;
      }

      if (type === 'function_call_output') {
        const output = typeof item.output === 'string' ? item.output : JSON.stringify(item.output || {});
        messages.push({ role: 'tool', tool_call_id: item.call_id, content: output });
        continue;
      }

      // A subagent answers long after its tool pair closed, so there is no slot to
      // reopen — it travels as an ordinary user turn, like in every other handler.
      if (type === 'subagent_result') {
        messages.push({ role: 'user', content: renderSubagentResult(item) });
        continue;
      }

      // `reasoning` is replayed as nothing: it is provider-local scratch space and
      // no compatible server accepts it back as input.
      if (type === 'reasoning') continue;

      const content = item.content;
      if (typeof content === 'string') {
        messages.push({ role: role === 'assistant' ? 'assistant' : 'user', content });
      }
    }

    return messages;
  }

  /** Converts a single tool execution result into its `tool`-role message. */
  convertToolOutput(output) {
    const content = typeof output.output === 'string' ? output.output : JSON.stringify(output.output || {});
    return { role: 'tool', tool_call_id: output.call_id, content };
  }

  /** Turns this turn's input (user text, or tool results + directive) into messages. */
  convertInputMessage(turnInput) {
    const messages = [];
    if (!turnInput) return messages;

    if (turnInput.type === 'message') {
      messages.push({ role: 'user', content: turnInput.content });
      return messages;
    }

    if (turnInput.type === 'subagent_result') {
      messages.push({ role: 'user', content: renderSubagentResult(turnInput.message) });
      return messages;
    }

    if (turnInput.type === 'tool_output') {
      for (const output of turnInput.items.filter((i) => i.type !== 'directive')) {
        messages.push(this.convertToolOutput(output));
      }
      const directive = turnInput.items.find((i) => i.type === 'directive');
      if (directive) {
        messages.push({ role: directive.role, content: directive.content });
      }
    }

    return messages;
  }

  /**
   * Parses one Chat Completions choice into canonical output items.
   *
   * Two hard-won invariants, both unconditional:
   *  1. Tool calls are detected by the PRESENCE of `message.tool_calls`, never by
   *     `finish_reason`. llama.cpp reports `finish_reason: "tool"`, not the spec's
   *     `"tool_calls"`, so gating on it drops every tool call that server makes.
   *  2. `arguments` is accepted as string OR object (llama.cpp#20198 returns the
   *     latter, violating the spec) and normalized to a string on the way in.
   */
  _parseResponse(response) {
    const output = [];
    const choice = response?.choices?.[0];
    if (!choice) {
      chatLogger.warn(`No choices in response from ${this.providerName}`);
      return output;
    }
    const message = choice.message || {};

    // Some servers expose the thinking trace as `reasoning_content` (Ollama, vLLM)
    // or `reasoning`. Captured rather than dropped so it survives into the transcript.
    const reasoningText = message.reasoning_content ?? message.reasoning;
    if (typeof reasoningText === 'string' && reasoningText.length > 0) {
      chatLogger.info(`[Reasoning] ${reasoningText.substring(0, 200)}...`);
      output.push({ type: 'reasoning', content: reasoningText, role: 'assistant' });
    }

    if (Array.isArray(message.tool_calls) && message.tool_calls.length > 0) {
      for (const tc of message.tool_calls) {
        const fn = tc.function || {};
        chatLogger.info(`✓ Tool call request: ${fn.name}`);
        output.push({
          type: 'function_call',
          name: fn.name,
          arguments: this._normalizeArguments(fn.arguments),
          call_id: tc.id || this._synthesizeCallId(),
        });
      }
    }

    if (typeof message.content === 'string' && message.content.length > 0) {
      chatLogger.info(`✓ Response: ${message.content.substring(0, 30)}...`);
      output.push({ type: 'text', content: message.content, role: 'assistant' });
    }

    if (output.length === 0) {
      chatLogger.warn(
        `Empty output from ${this.providerName} (finish_reason: ${choice.finish_reason ?? 'none'}). ` +
          'If this server needs a tool-enabled chat template, check it was started with --jinja.'
      );
    }

    return output;
  }

  async processMessage(turnInput = null, tools = [], conversationHistory = [], options = {}) {
    if (!this.client) {
      throw new Error(`${this.providerName} client not initialized`);
    }

    const { instructions = null, allowedTools = null, agent = null } = options;
    const profile = this.resolveModelConfig(agent);
    const modelId = profile.model || this.model;

    const messages = this.convertHistory(conversationHistory);
    messages.push(...this.convertInputMessage(turnInput));

    // The system prompt rides inline here (no `instructions` field in this protocol),
    // and only when the replayed history did not already carry one.
    const systemText = instructions || this.systemPrompt;
    if (systemText && !messages.some((m) => m.role === 'system')) {
      messages.unshift({ role: 'system', content: systemText });
    }

    const params = {
      model: modelId,
      messages,
      stream: false,
      ...this.capabilities.extraBody,
    };
    if (this.capabilities.temperature !== null) params.temperature = this.capabilities.temperature;
    if (this.capabilities.maxTokens) params.max_tokens = this.capabilities.maxTokens;
    if (this.capabilities.reasoningEffort) params.reasoning_effort = this.capabilities.reasoningEffort;

    // An empty `allowedTools` array is the orchestrator's way of forcing a text-only
    // answer, so tools are omitted entirely in that case.
    if (tools.length > 0 && (!allowedTools || allowedTools.length > 0)) {
      params.tools = this.convertToolsForMCP(tools);
      if (this.capabilities.parallelToolCalls !== null) {
        params.parallel_tool_calls = this.capabilities.parallelToolCalls;
      }
      if (this.capabilities.toolChoice) {
        params.tool_choice = 'auto';
      }
    }

    chatLogger.info('tools');
    for (const tool of tools) {
      chatLogger.info(`✓ ${tool.name}: ${tool.description.substring(0, 100)}...`);
    }
    chatLogger.info(`✓ Message for ${this.providerName}`);
    for (const msg of messages) {
      const text = typeof msg.content === 'string' ? msg.content : JSON.stringify(msg.content ?? msg.tool_calls ?? '');
      chatLogger.info(`- role: ${msg.role}, content: ${text.replace(/\r?\n|\r/g, ' ').substring(0, 100)}...`);
    }

    try {
      chatLogger.info(`→ Sending message to ${this.providerName} (model: ${modelId})...`);
      const response = await this.client.chat.completions.create(params);
      const output = this._parseResponse(response);

      return {
        output,
        responseId: response.id || null,
        model: response.model || modelId,
        status: 'completed',
        usage: this.normalizeUsage(response.usage),
      };
    } catch (error) {
      chatLogger.error(`Error in ${this.providerName}:`, error);
      throw error;
    }
  }

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

  /**
   * Chat Completions usage: { prompt_tokens, completion_tokens, total_tokens,
   * prompt_tokens_details.cached_tokens, completion_tokens_details.reasoning_tokens }.
   * Local servers usually emit only the first three.
   */
  normalizeUsage(rawUsage) {
    if (!rawUsage) return null;
    return makeUsage({
      input: rawUsage.prompt_tokens,
      output: rawUsage.completion_tokens,
      cached: rawUsage.prompt_tokens_details?.cached_tokens,
      reasoning: rawUsage.completion_tokens_details?.reasoning_tokens,
      total: rawUsage.total_tokens,
      raw: rawUsage,
    });
  }

  normalizeResponse(response) {
    const output = Array.isArray(response) ? response : response.output;
    return {
      provider: this.providerName,
      content: output,
      model: response.model || this.model,
      responseId: response.responseId || null,
      usage: response.usage || null,
      raw: response,
    };
  }

  getProviderName() {
    return this.providerName;
  }
}

export { OpenAICompatibleHandler };
