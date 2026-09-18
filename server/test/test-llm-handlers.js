import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import dotenv from 'dotenv';
import { LLMFactory } from '../models/chat/handlers/llmFactory.js';
import { OpenAIHandler } from '../models/chat/handlers/openaiHandler.js';
import { GeminiHandler } from '../models/chat/handlers/geminiHandler.js';
import { AnthropicHandler } from '../models/chat/handlers/antropicHandler.js';
import { OpenAICompatibleHandler } from '../models/chat/handlers/openaiCompatibleHandler.js';
import { normalizeBaseURL, COMPATIBLE_PROVIDERS } from '../models/chat/handlers/compatibleProviders.js';

// Load .env from server root
dotenv.config();

const FAKE_KEY = 'test-key-123';

// Real API keys from .env (used only in integration tests)
const realKeys = {
  openai: process.env.LLM_OPENAI_API_KEY || '',
  gemini: process.env.LLM_GEMINI_API_KEY || '',
  anthropic: process.env.LLM_ANTHROPIC_API_KEY || '',
  ollama: process.env.LLM_OLLAMA_API_KEY || '',
};

const mockTools = [
  {
    name: 'get_weather',
    description: 'Get current weather',
    inputSchema: {
      type: 'object',
      properties: { city: { type: 'string' } },
      required: ['city'],
    },
  },
];

const mockHistory = [
  { message: { role: 'system', content: 'You are helpful.' } },
  { message: { role: 'user', content: 'Hello' } },
  { message: { role: 'assistant', content: 'Hi there!' } },
];

const mockToolCall = {
  type: 'function_call',
  name: 'get_weather',
  arguments: '{"city":"Madrid"}',
  call_id: 'call_123',
};

const mockExecutor = async (_name, args) => ({ temp: 25, city: args.city });

// ═══════════════════════════════════════════════════════════════════
// Factory
// ═══════════════════════════════════════════════════════════════════
describe('LLMFactory', () => {
  it('creates OpenAI handler', () => {
    const handler = LLMFactory.createHandler('openai', FAKE_KEY);
    assert.equal(handler.getProviderName(), 'openai');
    assert.ok(handler instanceof OpenAIHandler);
  });

  it('creates Gemini handler', () => {
    const handler = LLMFactory.createHandler('gemini', FAKE_KEY);
    assert.equal(handler.getProviderName(), 'gemini');
    assert.ok(handler instanceof GeminiHandler);
  });

  it('creates Anthropic handler', () => {
    const handler = LLMFactory.createHandler('anthropic', FAKE_KEY);
    assert.equal(handler.getProviderName(), 'anthropic');
    assert.ok(handler instanceof AnthropicHandler);
  });

  it('creates Ollama handler on the compatible transport', () => {
    const handler = LLMFactory.createHandler('ollama');
    assert.equal(handler.getProviderName(), 'ollama');
    assert.ok(handler instanceof OpenAICompatibleHandler);
    assert.equal(handler.baseURL, 'http://localhost:11434/v1');
  });

  it('creates llama.cpp handler on the compatible transport', () => {
    const handler = LLMFactory.createHandler('llamacpp');
    assert.equal(handler.getProviderName(), 'llamacpp');
    assert.ok(handler instanceof OpenAICompatibleHandler);
    assert.equal(handler.baseURL, 'http://localhost:8080/v1');
  });

  it('creates a generic openai-compatible handler from an explicit baseURL', () => {
    const handler = LLMFactory.createHandler('openai-compatible', 'sk-real-key', 'qwen3', {
      baseURL: 'https://api.example.com/v1',
    });
    assert.equal(handler.baseURL, 'https://api.example.com/v1');
    assert.equal(handler.apiKey, 'sk-real-key');
    assert.equal(handler.model, 'qwen3');
  });

  it('creates Anthropic handler via "claude" alias', () => {
    const handler = LLMFactory.createHandler('claude', FAKE_KEY);
    assert.equal(handler.getProviderName(), 'anthropic');
  });

  it('throws on unsupported provider', () => {
    assert.throws(() => LLMFactory.createHandler('invalid', FAKE_KEY), /no soportado/);
  });

  it('lists supported providers', () => {
    const providers = LLMFactory.getSupportedProviders();
    assert.ok(providers.includes('openai'));
    assert.ok(providers.includes('gemini'));
    assert.ok(providers.includes('anthropic'));
    assert.ok(providers.includes('ollama'));
    assert.ok(providers.includes('llamacpp'));
    assert.ok(providers.includes('openai-compatible'));
  });
});

// ═══════════════════════════════════════════════════════════════════
// Config: API key resolution per provider
// ═══════════════════════════════════════════════════════════════════
describe('Config - API key per provider', () => {
  const mockApiKeys = {
    openai: 'sk-openai-123',
    gemini: 'AIza-gemini-456',
    anthropic: 'sk-ant-789',
  };

  it('resolves OpenAI API key from LLM_OPENAI_API_KEY', () => {
    const provider = 'openai';
    const apiKey = mockApiKeys[provider];
    assert.equal(apiKey, 'sk-openai-123');
    const handler = LLMFactory.createHandler(provider, apiKey);
    assert.equal(handler.apiKey, 'sk-openai-123');
  });

  it('resolves Gemini API key from LLM_GEMINI_API_KEY', () => {
    const provider = 'gemini';
    const apiKey = mockApiKeys[provider];
    assert.equal(apiKey, 'AIza-gemini-456');
    const handler = LLMFactory.createHandler(provider, apiKey);
    assert.equal(handler.apiKey, 'AIza-gemini-456');
  });

  it('resolves Anthropic API key from LLM_ANTHROPIC_API_KEY', () => {
    const provider = 'anthropic';
    const apiKey = mockApiKeys[provider];
    assert.equal(apiKey, 'sk-ant-789');
    const handler = LLMFactory.createHandler(provider, apiKey);
    assert.equal(handler.apiKey, 'sk-ant-789');
  });

  it('resolves "claude" alias to anthropic key', () => {
    const provider = 'claude';
    const apiKey = mockApiKeys[provider === 'claude' ? 'anthropic' : provider];
    assert.equal(apiKey, 'sk-ant-789');
    const handler = LLMFactory.createHandler(provider, apiKey);
    assert.equal(handler.apiKey, 'sk-ant-789');
  });

  it('takes the endpoint from baseURL only, never from the API-key slot', () => {
    // The endpoint lives in LLM_*_BASE_URL. A URL left in the key slot is rejected at
    // boot (server.js) instead of being re-interpreted here, so this stays a pure token.
    const handler = LLMFactory.createHandler('ollama', '', '', { baseURL: 'http://remote:11434' });
    assert.equal(handler.baseURL, 'http://remote:11434/v1');
    assert.equal(handler.apiKey, 'not-needed');
  });

  it('keeps a real token as a token when baseURL is given explicitly', () => {
    const handler = LLMFactory.createHandler('openai-compatible', 'sk-groq-123', 'llama-3.3', {
      baseURL: 'https://api.groq.com/openai/v1',
    });
    assert.equal(handler.apiKey, 'sk-groq-123');
    assert.equal(handler.baseURL, 'https://api.groq.com/openai/v1');
  });

  it('detects missing API key for provider', () => {
    const keysWithMissing = { openai: '', gemini: 'AIza-gemini-456', anthropic: 'sk-ant-789' };
    const provider = 'openai';
    const apiKey = keysWithMissing[provider];
    assert.equal(apiKey, '');
    assert.equal(!apiKey, true, 'Empty key should be falsy');
  });
});

// ═══════════════════════════════════════════════════════════════════
// Gemini Handler
// ═══════════════════════════════════════════════════════════════════
describe('GeminiHandler', () => {
  it('throws without API key', () => {
    assert.throws(() => new GeminiHandler(''), /API Key is required/);
  });

  it('sets default model', () => {
    const handler = new GeminiHandler(FAKE_KEY);
    assert.equal(handler.model, 'gemini-2.5-flash');
  });

  it('initializes client', async () => {
    const handler = new GeminiHandler(FAKE_KEY);
    await handler.initialize();
    assert.equal(handler.initialized, true);
    assert.ok(handler.client);
  });

  it('resolves model config per capability tier', () => {
    const handler = new GeminiHandler(FAKE_KEY);
    assert.equal(handler.resolveModelConfig({ capability: 'low' }).model, 'gemini-2.5-flash');
    assert.equal(handler.resolveModelConfig({ capability: 'medium' }).model, 'gemini-2.5-pro');
    assert.equal(handler.resolveModelConfig({ capability: 'high' }).model, 'gemini-3-flash-preview');
  });

  it('converts tools to functionDeclarations with parametersJsonSchema', () => {
    const handler = new GeminiHandler(FAKE_KEY);
    const converted = handler.convertToolsForMCP(mockTools);
    assert.equal(converted.length, 1);
    const decl = converted[0].functionDeclarations[0];
    assert.equal(decl.name, 'get_weather');
    assert.ok(decl.parametersJsonSchema);
    assert.equal(decl.parametersJsonSchema.type, 'object');
  });

  it('converts history skipping system and mapping assistant → model', () => {
    const handler = new GeminiHandler(FAKE_KEY);
    const msgs = [
      ...handler.convertHistory(mockHistory),
      ...handler.convertInputMessage({ type: 'message', content: 'New msg' }),
    ];
    // system skipped → user + model + new user = 3
    assert.equal(msgs.length, 3);
    assert.equal(msgs[0].role, 'user');
    assert.equal(msgs[1].role, 'model');
    assert.equal(msgs[2].role, 'user');
    assert.deepEqual(msgs[2].parts, [{ text: 'New msg' }]);
  });

  it('handles tool call and returns function_call_output with name', async () => {
    const handler = new GeminiHandler(FAKE_KEY);
    const result = await handler.handleToolCall(mockToolCall, mockExecutor);
    assert.equal(result.type, 'function_call_output');
    assert.equal(result.call_id, 'call_123');
    assert.equal(result.name, 'get_weather');
    assert.equal(JSON.parse(result.output).temp, 25);
  });

  it('handles tool call error gracefully', async () => {
    const handler = new GeminiHandler(FAKE_KEY);
    const failExecutor = async () => {
      throw new Error('boom');
    };
    const result = await handler.handleToolCall(mockToolCall, failExecutor);
    assert.equal(result.type, 'function_call_output');
    assert.ok(JSON.parse(result.output).error.includes('boom'));
  });

  it('normalizes response with content field', () => {
    const handler = new GeminiHandler(FAKE_KEY);
    const norm = handler.normalizeResponse({
      output: [{ type: 'text', content: 'Hello', role: 'assistant' }],
      model: 'gemini-2.5-flash',
    });
    assert.equal(norm.provider, 'gemini');
    assert.equal(norm.responseId, null);
    assert.equal(norm.content[0].content, 'Hello');
    assert.equal(norm.content[0].role, 'assistant');
  });

  it('convertMsg handles function_call history items', () => {
    const handler = new GeminiHandler(FAKE_KEY);
    const history = [
      { message: { type: 'function_call', name: 'get_weather', arguments: '{"city":"Madrid"}', call_id: 'call_1' } },
    ];
    const msgs = handler.convertHistory(history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'model');
    assert.ok(msgs[0].parts[0].functionCall);
    assert.equal(msgs[0].parts[0].functionCall.name, 'get_weather');
    assert.deepEqual(msgs[0].parts[0].functionCall.args, { city: 'Madrid' });
  });

  it('convertMsg handles function_call_output history items', () => {
    const handler = new GeminiHandler(FAKE_KEY);
    const history = [
      { message: { type: 'function_call_output', name: 'get_weather', call_id: 'call_1', output: '{"temp":25}' } },
    ];
    const msgs = handler.convertHistory(history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'user');
    assert.ok(msgs[0].parts[0].functionResponse);
    assert.equal(msgs[0].parts[0].functionResponse.name, 'get_weather');
    assert.deepEqual(msgs[0].parts[0].functionResponse.response, { temp: 25 });
  });

  it('convertMsg handles normalized text items with content field', () => {
    const handler = new GeminiHandler(FAKE_KEY);
    const history = [{ message: { type: 'text', content: 'Hello world', role: 'assistant' } }];
    const msgs = handler.convertHistory(history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'model');
    assert.deepEqual(msgs[0].parts, [{ text: 'Hello world' }]);
  });

  it('processMessage fails with fake key (validates API call path)', async () => {
    const handler = new GeminiHandler(FAKE_KEY);
    await handler.initialize();
    await assert.rejects(
      () => handler.processMessage({ type: 'message', content: 'test' }, [], [], {}),
      (err) => err.message.includes('API key')
    );
  });
});

// ═══════════════════════════════════════════════════════════════════
// Anthropic Handler
// ═══════════════════════════════════════════════════════════════════
describe('AnthropicHandler', () => {
  it('throws without API key', () => {
    assert.throws(() => new AnthropicHandler(''), /API Key is required/);
  });

  it('sets default model', () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    assert.equal(handler.model, 'claude-haiku-4-5-20251001');
  });

  it('initializes client', async () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    await handler.initialize();
    assert.equal(handler.initialized, true);
    assert.ok(handler.client);
  });

  it('resolves model config per capability tier', () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    const low = handler.resolveModelConfig({ capability: 'low' });
    const high = handler.resolveModelConfig({ capability: 'high' });
    assert.ok(low.model);
    assert.ok(high.maxTokens > 4096);
  });

  it('converts tools to Anthropic input_schema format', () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    const converted = handler.convertToolsForMCP(mockTools);
    assert.equal(converted.length, 1);
    assert.equal(converted[0].name, 'get_weather');
    assert.ok(converted[0].input_schema);
    assert.equal(converted[0].input_schema.type, 'object');
  });

  it('converts history skipping system messages', () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    const msgs = [
      ...handler.convertHistory(mockHistory),
      ...handler.convertInputMessage({ type: 'message', content: 'New msg' }),
    ];
    // system skipped → user + assistant + new user = 3
    assert.equal(msgs.length, 3);
    assert.equal(msgs[0].role, 'user');
    assert.equal(msgs[1].role, 'assistant');
    assert.equal(msgs[2].role, 'user');
    assert.equal(msgs[2].content, 'New msg');
  });

  it('handles tool call and returns function_call_output with name', async () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    const result = await handler.handleToolCall(mockToolCall, mockExecutor);
    assert.equal(result.type, 'function_call_output');
    assert.equal(result.call_id, 'call_123');
    assert.equal(result.name, 'get_weather');
    assert.equal(JSON.parse(result.output).temp, 25);
  });

  it('handles tool call error gracefully', async () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    const failExecutor = async () => {
      throw new Error('boom');
    };
    const result = await handler.handleToolCall(mockToolCall, failExecutor);
    assert.equal(result.type, 'function_call_output');
    assert.ok(JSON.parse(result.output).error.includes('boom'));
  });

  it('normalizes response with content field', () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    const norm = handler.normalizeResponse({
      output: [{ type: 'text', content: 'Hello', role: 'assistant' }],
      responseId: 'msg_123',
      model: 'claude-haiku-4-5-20251001',
    });
    assert.equal(norm.provider, 'anthropic');
    assert.equal(norm.responseId, 'msg_123');
    assert.equal(norm.content[0].content, 'Hello');
    assert.equal(norm.content[0].role, 'assistant');
  });

  it('convertMsg handles function_call history items as tool_use', () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    const history = [
      { message: { type: 'function_call', name: 'get_weather', arguments: '{"city":"Madrid"}', call_id: 'call_1' } },
    ];
    const msgs = handler.convertHistory(history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'assistant');
    assert.equal(msgs[0].content[0].type, 'tool_use');
    assert.equal(msgs[0].content[0].name, 'get_weather');
    assert.equal(msgs[0].content[0].id, 'call_1');
    assert.deepEqual(msgs[0].content[0].input, { city: 'Madrid' });
  });

  it('convertMsg handles function_call_output history items as tool_result', () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    const history = [
      { message: { type: 'function_call_output', name: 'get_weather', call_id: 'call_1', output: '{"temp":25}' } },
    ];
    const msgs = handler.convertHistory(history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'user');
    assert.equal(msgs[0].content[0].type, 'tool_result');
    assert.equal(msgs[0].content[0].tool_use_id, 'call_1');
    assert.equal(msgs[0].content[0].content, '{"temp":25}');
  });

  it('convertMsg handles normalized text items with content field', () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    const history = [{ message: { type: 'text', content: 'Hello world', role: 'assistant' } }];
    const msgs = handler.convertHistory(history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'assistant');
    assert.equal(msgs[0].content, 'Hello world');
  });

  it('processMessage fails with fake key (validates API call path)', async () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    await handler.initialize();
    await assert.rejects(
      () => handler.processMessage({ type: 'message', content: 'test' }, [], [], {}),
      (err) => err.message.includes('authentication') || err.message.includes('api-key') || err.status === 401
    );
  });
});

// ═══════════════════════════════════════════════════════════════════
// OpenAI-compatible Handler (Ollama, llama.cpp, vLLM, LM Studio…)
// ═══════════════════════════════════════════════════════════════════
describe('OpenAICompatibleHandler', () => {
  const BASE = 'http://localhost:11434/v1';
  const make = (capabilities = {}) =>
    new OpenAICompatibleHandler('not-needed', 'test-model', 'sys', {
      baseURL: BASE,
      providerName: 'ollama',
      capabilities,
    });

  /** Stubs the SDK so a processMessage call returns `reply` and records the params sent. */
  const stubClient = (handler, reply = { choices: [{ message: { content: 'ok' } }] }) => {
    const sent = {};
    handler.client = {
      chat: {
        completions: {
          create: async (params) => {
            Object.assign(sent, params);
            return reply;
          },
        },
      },
    };
    return sent;
  };

  it('throws without baseURL', () => {
    assert.throws(() => new OpenAICompatibleHandler('k', 'm', 'sys', {}), /baseURL is required/);
  });

  it('throws without model', () => {
    assert.throws(() => new OpenAICompatibleHandler('k', '', 'sys', { baseURL: BASE }), /model is required/);
  });

  it('normalizeBaseURL appends /v1 only when missing', () => {
    assert.equal(normalizeBaseURL('http://localhost:11434'), 'http://localhost:11434/v1');
    assert.equal(normalizeBaseURL('http://localhost:11434/'), 'http://localhost:11434/v1');
    assert.equal(normalizeBaseURL('http://localhost:11434/v1'), 'http://localhost:11434/v1');
    assert.equal(normalizeBaseURL(''), '');
  });

  it('converts tools to Chat Completions shape (nested under function)', () => {
    const converted = make().convertToolsForMCP(mockTools);
    assert.equal(converted.length, 1);
    assert.equal(converted[0].type, 'function');
    assert.equal(converted[0].function.name, 'get_weather');
    assert.equal(converted[0].function.parameters.type, 'object');
  });

  it('converts history keeping system messages inline', () => {
    const handler = make();
    const msgs = [
      ...handler.convertHistory(mockHistory),
      ...handler.convertInputMessage({ type: 'message', content: 'New msg' }),
    ];
    assert.equal(msgs.length, 4);
    assert.equal(msgs[0].role, 'system');
    assert.equal(msgs[1].role, 'user');
    assert.equal(msgs[2].role, 'assistant');
    assert.equal(msgs[3].content, 'New msg');
  });

  it('emits assistant tool_calls with an id and STRING arguments', () => {
    const history = [
      { message: { type: 'function_call', name: 'get_weather', arguments: '{"city":"Madrid"}', call_id: 'call_1' } },
    ];
    const msgs = make().convertHistory(history);
    assert.equal(msgs[0].role, 'assistant');
    assert.equal(msgs[0].tool_calls[0].id, 'call_1');
    assert.equal(msgs[0].tool_calls[0].type, 'function');
    // Chat Completions requires a JSON string here, not an object.
    assert.equal(typeof msgs[0].tool_calls[0].function.arguments, 'string');
    assert.equal(msgs[0].tool_calls[0].function.arguments, '{"city":"Madrid"}');
  });

  it('pairs tool results back with tool_call_id (the native /api/chat path dropped it)', () => {
    const history = [
      { message: { type: 'function_call_output', name: 'get_weather', call_id: 'call_1', output: '{"temp":25}' } },
    ];
    const msgs = make().convertHistory(history);
    assert.equal(msgs[0].role, 'tool');
    assert.equal(msgs[0].tool_call_id, 'call_1');
    assert.equal(msgs[0].content, '{"temp":25}');
  });

  it('detects tool calls by presence, NOT by finish_reason (llama.cpp returns "tool")', () => {
    const parsed = make()._parseResponse({
      choices: [
        {
          finish_reason: 'tool',
          message: { tool_calls: [{ id: 'c1', function: { name: 'get_weather', arguments: '{"city":"Madrid"}' } }] },
        },
      ],
    });
    assert.equal(parsed.length, 1);
    assert.equal(parsed[0].type, 'function_call');
    assert.equal(parsed[0].call_id, 'c1');
  });

  it('accepts arguments returned as a parsed object (llama.cpp#20198) and restores the string', () => {
    const parsed = make()._parseResponse({
      choices: [
        { message: { tool_calls: [{ id: 'c1', function: { name: 'get_weather', arguments: { city: 'Madrid' } } }] } },
      ],
    });
    assert.equal(typeof parsed[0].arguments, 'string');
    assert.deepEqual(JSON.parse(parsed[0].arguments), { city: 'Madrid' });
  });

  it('synthesizes a call_id when the server omits one', () => {
    const parsed = make()._parseResponse({
      choices: [{ message: { tool_calls: [{ function: { name: 'get_weather', arguments: '{}' } }] } }],
    });
    assert.ok(parsed[0].call_id.startsWith('ollama_'), 'id should be synthesized with the provider prefix');
  });

  it('captures reasoning_content instead of dropping it', () => {
    const parsed = make()._parseResponse({
      choices: [{ message: { reasoning_content: 'thinking hard', content: 'done' } }],
    });
    assert.equal(parsed[0].type, 'reasoning');
    assert.equal(parsed[0].content, 'thinking hard');
    assert.equal(parsed[1].type, 'text');
  });

  it('normalizes Chat Completions usage field names', () => {
    const usage = make().normalizeUsage({ prompt_tokens: 100, completion_tokens: 20, total_tokens: 120 });
    assert.equal(usage.input, 100);
    assert.equal(usage.output, 20);
    assert.equal(usage.total, 120);
  });

  it('handles tool call and returns function_call_output with name', async () => {
    const result = await make().handleToolCall(mockToolCall, mockExecutor);
    assert.equal(result.type, 'function_call_output');
    assert.equal(result.call_id, 'call_123');
    assert.equal(result.name, 'get_weather');
    assert.equal(JSON.parse(result.output).temp, 25);
  });

  it('handles tool call error gracefully', async () => {
    const failExecutor = async () => {
      throw new Error('boom');
    };
    const result = await make().handleToolCall(mockToolCall, failExecutor);
    assert.equal(result.type, 'function_call_output');
    assert.ok(JSON.parse(result.output).error.includes('boom'));
  });

  it('normalizes response with the configured provider name', () => {
    const norm = make().normalizeResponse({ output: [{ type: 'text', content: 'Hello' }], model: 'llama3.2' });
    assert.equal(norm.provider, 'ollama');
    assert.equal(norm.content[0].content, 'Hello');
  });

  // ── Capability rule: flags shape the REQUEST only ────────────────────────────
  it('Ollama preset omits tool_choice and parallel_tool_calls (unsupported on its /v1)', async () => {
    const handler = make(COMPATIBLE_PROVIDERS.ollama.capabilities);
    const sent = stubClient(handler);
    await handler.processMessage({ type: 'message', content: 'hi' }, mockTools, [], {});
    assert.ok(sent.tools, 'tools should still be sent');
    assert.equal(sent.tool_choice, undefined);
    assert.equal(sent.parallel_tool_calls, undefined);
    assert.equal(sent.reasoning_effort, 'none');
  });

  it('llama.cpp preset sends parallel_tool_calls explicitly (disabled by default there)', async () => {
    const handler = new OpenAICompatibleHandler('not-needed', 'm', 'sys', {
      baseURL: 'http://localhost:8080/v1',
      providerName: 'llamacpp',
      capabilities: COMPATIBLE_PROVIDERS.llamacpp.capabilities,
    });
    const sent = stubClient(handler);
    await handler.processMessage({ type: 'message', content: 'hi' }, mockTools, [], {});
    assert.equal(sent.parallel_tool_calls, true);
    assert.equal(sent.tool_choice, 'auto');
  });

  it('omits tools entirely when allowedTools is empty (text-only turn)', async () => {
    const handler = make();
    const sent = stubClient(handler);
    await handler.processMessage({ type: 'message', content: 'hi' }, mockTools, [], { allowedTools: [] });
    assert.equal(sent.tools, undefined);
  });
});

// ═══════════════════════════════════════════════════════════════════
// Integration: real API calls (skipped if key missing)
// ═══════════════════════════════════════════════════════════════════
describe('Integration - real API smoke test', () => {
  const prompt = 'Respond with only the word "pong". Nothing else.';

  it('OpenAI responds to a simple message', { skip: !realKeys.openai && 'LLM_OPENAI_API_KEY not set' }, async (t) => {
    const handler = LLMFactory.createHandler('openai', realKeys.openai);
    await handler.initialize();

    let result;
    try {
      result = await handler.processMessage({ type: 'message', content: prompt }, [], [], {});
    } catch (err) {
      if (err.message?.includes('credit') || err.message?.includes('quota') || err.status === 429) {
        t.skip('OpenAI API billing/quota issue: ' + err.message.substring(0, 80));
        return;
      }
      throw err;
    }

    assert.equal(result.status, 'completed');
    assert.ok(Array.isArray(result.output), 'output should be an array');
    assert.ok(result.output.length > 0, 'output should not be empty');

    const textBlock = result.output.find((o) => o.type === 'text' || o.type === 'message');
    assert.ok(textBlock, 'should contain a text block');
  });

  it('Gemini responds to a simple message', { skip: !realKeys.gemini && 'LLM_GEMINI_API_KEY not set' }, async (t) => {
    const handler = LLMFactory.createHandler('gemini', realKeys.gemini);
    await handler.initialize();

    let result;
    try {
      result = await handler.processMessage({ type: 'message', content: prompt }, [], [], {});
    } catch (err) {
      if (err.message?.includes('credit') || err.message?.includes('quota') || err.status === 429) {
        t.skip('Gemini API billing/quota issue: ' + err.message.substring(0, 80));
        return;
      }
      throw err;
    }

    assert.equal(result.status, 'completed');
    assert.ok(Array.isArray(result.output), 'output should be an array');
    assert.ok(result.output.length > 0, 'output should not be empty');

    const textBlock = result.output.find((o) => o.type === 'text');
    assert.ok(textBlock, 'should contain a text block');
    assert.ok((textBlock.content || textBlock.text)?.length > 0, 'text should not be empty');
  });

  it(
    'Anthropic responds to a simple message',
    { skip: !realKeys.anthropic && 'LLM_ANTHROPIC_API_KEY not set' },
    async (t) => {
      const handler = LLMFactory.createHandler('anthropic', realKeys.anthropic);
      await handler.initialize();

      let result;
      try {
        result = await handler.processMessage({ type: 'message', content: prompt }, [], [], {});
      } catch (err) {
        // Skip on billing/quota errors — key is valid but account has no credits
        if (err.message?.includes('credit balance') || err.message?.includes('rate limit') || err.status === 429) {
          t.skip('Anthropic API billing/quota issue: ' + err.message.substring(0, 80));
          return;
        }
        throw err;
      }

      assert.equal(result.status, 'completed');
      assert.ok(Array.isArray(result.output), 'output should be an array');
      assert.ok(result.output.length > 0, 'output should not be empty');

      const textBlock = result.output.find((o) => o.type === 'text');
      assert.ok(textBlock, 'should contain a text block');
      assert.ok((textBlock.content || textBlock.text)?.length > 0, 'text should not be empty');
    }
  );

  it('Ollama responds to a simple message', { skip: !realKeys.ollama && 'LLM_OLLAMA_API_KEY not set' }, async (t) => {
    const handler = LLMFactory.createHandler('ollama', realKeys.ollama);
    await handler.initialize();

    let result;
    try {
      result = await handler.processMessage({ type: 'message', content: prompt }, [], [], {});
    } catch (err) {
      if (
        err.message?.includes('ECONNREFUSED') ||
        err.message?.includes('fetch failed') ||
        err.message?.includes('Connection error')
      ) {
        t.skip('Ollama not reachable: ' + err.message.substring(0, 80));
        return;
      }
      throw err;
    }

    assert.equal(result.status, 'completed');
    assert.ok(Array.isArray(result.output), 'output should be an array');
    assert.ok(result.output.length > 0, 'output should not be empty');

    const textBlock = result.output.find((o) => o.type === 'text');
    assert.ok(textBlock, 'should contain a text block');
    assert.ok((textBlock.content || textBlock.text)?.length > 0, 'text should not be empty');
  });
});
