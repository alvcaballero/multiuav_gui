import { GoogleGenAI } from '@google/genai';
import { BaseLLMHandler, FORCE_FINISH_MESSAGE, makeUsage, renderSubagentResult } from './baseLLMhandler.js';
import { SystemPrompts } from '../agents/index.js';
import { logger, chatLogger } from '../../../common/logger.js';

// Models: gemini-2.5-flash, gemini-2.5-pro, gemini-2.0-flash, gemini-3-flash-preview, gemini-3.1-pro-preview
class GeminiHandler extends BaseLLMHandler {
  static CAPABILITY_MAP = {
    low: { model: 'gemini-2.5-flash' },
    medium: { model: 'gemini-2.5-pro' },
    high: { model: 'gemini-3-flash-preview' },
  };

  constructor(apiKey, model = 'gemini-2.5-flash', systemPrompt = SystemPrompts.main) {
    super(apiKey, model, systemPrompt);
    if (!apiKey) {
      throw new Error('Google API Key is required for GeminiHandler.');
    }
  }

  async initialize() {
    this.client = new GoogleGenAI({ apiKey: this.apiKey });
    this.initialized = true;
    chatLogger.info(`✓ Gemini client initialized (model: ${this.model})`);
  }

  /**
   * Converts MCP tools to Gemini functionDeclarations format.
   * Uses parametersJsonSchema (required by @google/genai v1.x).
   */
  convertToolsForMCP(tools) {
    return [
      {
        functionDeclarations: tools.map((tool) => ({
          name: tool.name,
          description: tool.description,
          parametersJsonSchema: tool.inputSchema,
        })),
      },
    ];
  }

  /**
   * Converts conversation history to Gemini's contents format.
   * Gemini uses 'user' and 'model' roles (not 'assistant').
   */
  convertMsg(message = null, conversationHistory) {
    const contents = [];

    for (const msg of conversationHistory) {
      const item = msg.message || msg;
      const role = item.role;
      const type = item.type;

      // Skip system messages — handled via systemInstruction
      if (role === 'system') continue;

      // Async subagent result → plain user text part (no functionResponse to pair)
      if (type === 'subagent_result') {
        contents.push({ role: 'user', parts: [{ text: renderSubagentResult(item) }] });
        continue;
      }

      // Normalized function_call from DB → Gemini functionCall part (model role)
      if (type === 'function_call') {
        let args = {};
        try {
          args = typeof item.arguments === 'string' ? JSON.parse(item.arguments) : item.arguments || {};
        } catch {
          /* keep empty */
        }
        const part = { functionCall: { name: item.name, args } };
        // Restore thoughtSignature for Gemini thinking models (required to avoid 400 errors)
        if (item.thoughtSignature) {
          part.thoughtSignature = item.thoughtSignature;
        }
        contents.push({
          role: 'model',
          parts: [part],
        });
        continue;
      }

      // Normalized function_call_output from DB → Gemini functionResponse part (user role)
      if (type === 'function_call_output') {
        let response = {};
        let args = {};
        try {
          let output = typeof item.output === 'string' ? JSON.parse(item.output) : item.output || {};
          try {
            response =
              typeof output.content?.[0]?.text === 'string'
                ? JSON.parse(output.content[0].text)
                : output.content?.[0]?.text || output.content || output;
          } catch {
            response = { text: output.content?.[0]?.text || output.content?.[0] || output };
          }

          if (item.call_id) {
            args.call_id = item.call_id; // Preserve call_id for matching responses to tool calls
          }
        } catch {
          /* keep empty */
        }

        const parts = [{ functionResponse: { name: item.name, response } }];

        // Si la respuesta contiene datos de imagen, añadimos una parte de imagen para que el VLM la analice
        if (response && response.image_data) {
          parts.push({
            inlineData: {
              mimeType: response.mime_type || 'image/jpeg',
              data: response.image_data,
            },
          });
        }

        contents.push({
          role: 'user',
          parts: parts,
        });
        // console.log('Parsed function call output response:', response);
        continue;
      }

      // Text content (normalized format with `content` or legacy string)
      const content = item.content;
      const geminiRole = role === 'assistant' ? 'model' : 'user';

      if (typeof content === 'string') {
        const part = { text: content };
        if (item.thoughtSignature) {
          part.thoughtSignature = item.thoughtSignature;
        }
        contents.push({ role: geminiRole, parts: [part] });
      }
    }

    if (message !== null) {
      contents.push({ role: 'user', parts: [{ text: message }] });
    }

    return contents;
  }

  /**
   * Converts a single tool execution result to Gemini's message parts
   * (functionResponse + optional inlineData for images).
   */
  convertToolOutput(output) {
    const rawResponse = output.output;
    const jsonresponse = typeof rawResponse === 'string' ? JSON.parse(rawResponse) : rawResponse;
    let parsedResponse;
    try {
      parsedResponse =
        typeof jsonresponse.content?.[0]?.text === 'string'
          ? JSON.parse(jsonresponse.content[0].text)
          : jsonresponse.content?.[0]?.text || jsonresponse.content || jsonresponse;
    } catch {
      parsedResponse = {
        text: jsonresponse.content?.[0]?.text || jsonresponse.content?.[0] || jsonresponse,
      };
    }

    const parts = [
      {
        functionResponse: {
          name: output.name,
          response: parsedResponse || {},
        },
      },
    ];

    // Soporte para imágenes en la continuación del tool loop
    if (parsedResponse && parsedResponse.image_data) {
      parts.push({
        inlineData: {
          mimeType: parsedResponse.mime_type || 'image/jpeg',
          data: parsedResponse.image_data,
        },
      });
    }

    return parts;
  }

  /**
   * Parses Gemini response into the normalized output array format
   * that the orchestrator expects (matching OpenAI's output structure).
   */
  _parseGeminiResponse(response) {
    const output = [];

    const usage = response.usageMetadata || {};
    chatLogger.info(`Thoughts tokens: ${usage.thoughtsTokenCount ?? 0}`);
    chatLogger.info(`Output tokens: ${usage.candidatesTokenCount ?? 0}`);
    chatLogger.info(`Total tokens: ${usage.totalTokenCount ?? 0}`);

    chatLogger.info('Parsing Gemini response...');
    chatLogger.info(`✓ Full response: ${JSON.stringify(response).substring(0, 1000000)}...`);
    chatLogger.info(`✓ Candidates: ${response.candidates ? response.candidates.length : 0}`);
    chatLogger.info(`✓ response text: ${response.text}`);

    const candidates = response.candidates || [];
    for (const candidate of candidates) {
      // Handle MALFORMED_FUNCTION_CALL: model tried to call a tool but generated invalid JSON args.
      // Treat it as a recoverable error and return a text fallback so the orchestrator doesn't hang.
      if (candidate.finishReason === 'MALFORMED_FUNCTION_CALL') {
        chatLogger.warn(`⚠ Gemini returned MALFORMED_FUNCTION_CALL — injecting fallback text response`);
        output.push({
          type: 'text',
          content:
            'I encountered an internal error while trying to use a tool. Please rephrase your request or try again.',
          role: 'assistant',
        });
        continue;
      }

      const parts = candidate.content?.parts || [];
      for (const part of parts) {
        if (part.functionCall) {
          chatLogger.info(`✓ Tool call request: ${part.functionCall.name}`);
          const fcEntry = {
            type: 'function_call',
            name: part.functionCall.name,
            arguments: JSON.stringify(part.functionCall.args || {}),
            call_id: `gemini_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`,
          };
          // Preserve thoughtSignature for Gemini thinking models (required for history replay)
          if (part.thoughtSignature) {
            fcEntry.thoughtSignature = part.thoughtSignature;
          }
          output.push(fcEntry);
        } else if (part.text) {
          chatLogger.info(`✓ Response: ${part.text.substring(0, 30)}...`);
          const textEntry = {
            type: 'text',
            content: part.text,
            role: 'assistant',
          };
          // Preserve thoughtSignature for text parts too (recommended by Gemini docs)
          if (part.thoughtSignature) {
            textEntry.thoughtSignature = part.thoughtSignature;
          }
          output.push(textEntry);
        }
      }
    }

    // Fallback: use response.text if no candidates parsed
    if (output.length === 0 && response.text) {
      output.push({ type: 'text', content: response.text, role: 'assistant' });
    }
    chatLogger.info(`✓ Parsed ${output.length} output parts from Gemini response`);

    return output;
  }

  /**
   * Processes a message following the same interface as OpenAIHandler.
   * Returns { output: Array, responseId: string|null, model: string, status: string }
   */
  async processMessage(message = null, tools = [], conversationHistory = [], options = {}) {
    if (!this.client) {
      throw new Error('Gemini client not initialized');
    }

    const { instructions = null, toolOutputs = null, allowedTools = null, forceFinish = false, agent = null } = options;

    const profile = this.resolveModelConfig(agent);
    const modelId = profile.model || this.model;

    // Build config
    const config = { temperature: 1 }; // Adjust temperature as needed

    // System instruction
    const systemText = instructions || this.systemPrompt;
    if (systemText) {
      config.systemInstruction = systemText;
      logger.info(`✓ Using system instruction: ${systemText.substring(0, 100)}...`);
    }
    // Add tools if available (empty allowedTools array = no tools for forced text response)
    if (tools.length > 0 && (!allowedTools || allowedTools.length > 0)) {
      config.tools = this.convertToolsForMCP(tools);
    }

    // Build contents (conversation history)
    let contents;

    if (toolOutputs && toolOutputs.length > 0) {
      // Tool continuation: build history + function responses
      contents = this.convertMsg(null, conversationHistory);

      // Convert tool outputs to Gemini FunctionResponse format
      const functionResponses = [];
      for (const output of toolOutputs) {
        functionResponses.push(...this.convertToolOutput(output));
      }
      contents.push({ role: 'user', parts: functionResponses });

      if (forceFinish) {
        contents.push({
          role: 'user',
          parts: [{ text: FORCE_FINISH_MESSAGE }],
        });
      }
    } else if (message !== null) {
      contents = this.convertMsg(message, conversationHistory);
    } else {
      contents = this.convertMsg(null, conversationHistory);
    }

    chatLogger.info('Tools');
    for (const tool of tools) {
      chatLogger.info(`✓ ${tool.name}: ${tool.description.substring(0, 100)}...`);
    }
    chatLogger.info(`✓ Message for Gemini`);
    for (const msg of contents) {
      const summary = JSON.stringify(msg.parts || msg.content || '')
        .replace(/\r?\n|\r/g, ' ')
        .substring(0, 120);
      chatLogger.info(`- role: ${msg.role}, parts: ${summary}...`);
    }

    try {
      chatLogger.info(`→ Sending message to Gemini (model: ${modelId})...`);

      const response = await this.client.models.generateContent({
        model: modelId,
        contents,
        config,
      });

      const output = this._parseGeminiResponse(response);

      return {
        output,
        responseId: null, // Gemini doesn't have persistent response IDs
        model: modelId,
        status: 'completed',
        usage: this.normalizeUsage(response.usageMetadata),
      };
    } catch (error) {
      chatLogger.error('Error in Gemini:', error);
      throw error;
    }
  }

  /**
   * Handles a tool call from Gemini, matching OpenAI's output format
   * so the orchestrator can pass it back as toolOutputs.
   */
  async handleToolCall(toolCall, toolExecutor) {
    chatLogger.info('Handling tool call:', JSON.stringify(toolCall, null, 2).substring(0, 30) + '...');
    try {
      const functionName = toolCall.name;
      const functionArgs = JSON.parse(toolCall.arguments);

      const result = await toolExecutor(functionName, functionArgs);
      chatLogger.info(`✓ Tool ${functionName} response:`, JSON.stringify(result, null, 2).substring(0, 30) + '...');

      // Return format matching OpenAI's function_call_output
      // but with 'name' for Gemini FunctionResponse conversion
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
   * Gemini: usageMetadata = { promptTokenCount, candidatesTokenCount,
   * thoughtsTokenCount, cachedContentTokenCount, totalTokenCount }.
   * promptTokenCount already includes image tokens (promptTokensDetails breaks
   * them down by modality when present — kept in `raw`).
   * Output is derived from the provider's own total so we don't have to guess
   * whether thoughts are counted inside candidates or on top of them.
   */
  normalizeUsage(rawUsage) {
    if (!rawUsage) return null;
    const n = (v) => (Number.isFinite(Number(v)) ? Number(v) : 0);
    const input = n(rawUsage.promptTokenCount);
    const total = n(rawUsage.totalTokenCount);
    const candidates = n(rawUsage.candidatesTokenCount);
    const thoughts = n(rawUsage.thoughtsTokenCount);
    return makeUsage({
      input,
      output: total > input ? total - input : candidates + thoughts,
      cached: rawUsage.cachedContentTokenCount,
      reasoning: thoughts,
      total: total || undefined,
      raw: rawUsage,
    });
  }

  normalizeResponse(response) {
    const output = Array.isArray(response) ? response : response.output;

    return {
      provider: 'gemini',
      content: output,
      model: response.model || this.model,
      responseId: null,
      usage: response.usage || null,
      raw: response,
    };
  }

  getProviderName() {
    return 'gemini';
  }
}

export { GeminiHandler };
