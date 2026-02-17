import { GoogleGenAI } from '@google/genai';
import { BaseLLMHandler } from './baseLLMhandler.js';
import { SystemPrompts } from './prompts/index.js';
import logger, { chatLogger } from '../../common/logger.js';

// Models: gemini-2.5-flash, gemini-2.5-pro, gemini-2.0-flash, gemini-3-flash-preview, etc.
class GeminiHandler extends BaseLLMHandler {
  static AGENT_PROFILES = {
    default: {
      model: 'gemini-2.5-flash',
    },
    planner: {
      model: 'gemini-3-flash-preview',
    },
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

      // Normalized function_call from DB → Gemini functionCall part (model role)
      if (type === 'function_call') {
        let args = {};
        try {
          args = typeof item.arguments === 'string' ? JSON.parse(item.arguments) : (item.arguments || {});
        } catch { /* keep empty */ }
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
        try {
          response = typeof item.output === 'string' ? JSON.parse(item.output) : (item.output || {});
        } catch { /* keep empty */ }
        contents.push({
          role: 'user',
          parts: [{ functionResponse: { name: item.name, response } }],
        });
        continue;
      }

      // Text content (normalized format with `content` or legacy string)
      const content = item.content;
      const geminiRole = role === 'assistant' ? 'model' : 'user';

      if (typeof content === 'string') {
        contents.push({ role: geminiRole, parts: [{ text: content }] });
      }
    }

    if (message !== null) {
      contents.push({ role: 'user', parts: [{ text: message }] });
    }

    return contents;
  }

  /**
   * Parses Gemini response into the normalized output array format
   * that the orchestrator expects (matching OpenAI's output structure).
   */
  _parseGeminiResponse(response) {
    const output = [];
     
    // ...
    chatLogger.info(`Thoughts tokens: ${response.usageMetadata.thoughtsTokenCount}`);
    chatLogger.info(`Output tokens: ${response.usageMetadata.candidatesTokenCount}`);
    chatLogger.info(`Total tokens: ${response.usageMetadata.totalTokenCount}`);

    chatLogger.info('Parsing Gemini response...');
    chatLogger.info(`✓ Full response: ${JSON.stringify(response).substring(0, 1000000)}...`);
    chatLogger.info(`✓ Candidates: ${response.candidates ? response.candidates.length : 0}`);
    chatLogger.info(`✓ response text: ${response.text}`);

    if (response.functionCalls && response.functionCalls.length > 0) {
      for (const funcCall of response.functionCalls) {
        chatLogger.info(`✓ Tool call request: ${funcCall.name}`);
        // output.push({
        //   type: 'function_call',
        //   name: funcCall.name,
        //   arguments: JSON.stringify(funcCall.args || {}),
        //   call_id: `gemini_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`,
        // });
      }
    }

    const candidates = response.candidates || [];
    for (const candidate of candidates) {
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

    const {
      instructions = null,
      toolOutputs = null,
      allowedTools = null,
      forceFinish = false,
      agentProfile = 'default',
    } = options;

    const profile = this.getAgentProfile(agentProfile);
    const modelId = profile.model || this.model;

    // Build config
    const config = {};

    // System instruction
    const systemText = instructions || this.systemPrompt;
    if (systemText) {
     config.systemInstruction = systemText;
     logger.info(`✓ Using system instruction: ${systemText.substring(0, 100)}...`);
    }
    // Prepend system prompt only if not already present in conversation history
    // const systemText = instructions || this.systemPrompt;
    // const hasSystemMessage = messages.some((m) => m.role === 'system');
    // if (systemText && !hasSystemMessage) {
    //   messages.unshift({ role: 'system', content: systemText });
    // }


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
      const functionResponses = toolOutputs.map((output) => {
        let parsedResponse = output.output;
        if (typeof parsedResponse === 'string') {
          try {
            parsedResponse = JSON.parse(parsedResponse);
          } catch {
            parsedResponse = { raw: parsedResponse };
          }
        }
        return {
          functionResponse: {
            name: output.name,
            response: parsedResponse || {},
          },
        };
      });
      contents.push({ role: 'user', parts: functionResponses });

      if (forceFinish) {
        contents.push({
          role: 'user',
          parts: [
            {
              text: 'Maximum tool iterations reached. You MUST provide your final response NOW using only the information gathered so far. Do NOT attempt to call any more tools.',
            },
          ],
        });
      }
    } else if (message !== null) {
      contents = this.convertMsg(message, conversationHistory);
    } else {
      contents = this.convertMsg(null, conversationHistory);
    }
    
    chatLogger.info('Tools')
    for (const tool of tools) {
      chatLogger.info(`✓ ${tool.name}: ${tool.description.substring(0, 100)}...`);
    }
    chatLogger.info(`✓ Message for Gemini`);
    for (const msg of contents) {
      const summary = JSON.stringify(msg.parts || msg.content || '').replace(/\r?\n|\r/g, ' ').substring(0, 100);
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

  normalizeResponse(response) {
    const output = Array.isArray(response) ? response : response.output;

    return {
      provider: 'gemini',
      content: output,
      model: response.model || this.model,
      responseId: null,
      raw: response,
    };
  }

  getProviderName() {
    return 'gemini';
  }
}

export { GeminiHandler };
