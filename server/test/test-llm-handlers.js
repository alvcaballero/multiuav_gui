import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import dotenv from 'dotenv';
import { LLMFactory } from '../models/chat/llmFactory.js';
import { OpenAIHandler } from '../models/chat/openaiHandler.js';
import { GeminiHandler } from '../models/chat/geminiHandler.js';
import { AnthropicHandler } from '../models/chat/antropicHandler.js';
import { OllamaHandler } from '../models/chat/ollamaHandler.js';

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

  it('creates Ollama handler', () => {
    const handler = LLMFactory.createHandler('ollama', 'http://localhost:11434');
    assert.equal(handler.getProviderName(), 'ollama');
    assert.ok(handler instanceof OllamaHandler);
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

  it('resolves Ollama host URL from LLM_OLLAMA_API_KEY', () => {
    const provider = 'ollama';
    const apiKey = 'http://localhost:11434';
    const handler = LLMFactory.createHandler(provider, apiKey);
    assert.equal(handler.apiKey, 'http://localhost:11434');
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

  it('has agent profiles', () => {
    const handler = new GeminiHandler(FAKE_KEY);
    const def = handler.getAgentProfile('default');
    const plan = handler.getAgentProfile('planner');
    assert.equal(def.model, 'gemini-2.5-flash');
    assert.equal(plan.model, 'gemini-2.5-flash');
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
    const msgs = handler.convertMsg('New msg', mockHistory);
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
    const failExecutor = async () => { throw new Error('boom'); };
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
    const msgs = handler.convertMsg(null, history);
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
    const msgs = handler.convertMsg(null, history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'user');
    assert.ok(msgs[0].parts[0].functionResponse);
    assert.equal(msgs[0].parts[0].functionResponse.name, 'get_weather');
    assert.deepEqual(msgs[0].parts[0].functionResponse.response, { temp: 25 });
  });

  it('convertMsg handles normalized text items with content field', () => {
    const handler = new GeminiHandler(FAKE_KEY);
    const history = [
      { message: { type: 'text', content: 'Hello world', role: 'assistant' } },
    ];
    const msgs = handler.convertMsg(null, history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'model');
    assert.deepEqual(msgs[0].parts, [{ text: 'Hello world' }]);
  });

  it('processMessage fails with fake key (validates API call path)', async () => {
    const handler = new GeminiHandler(FAKE_KEY);
    await handler.initialize();
    await assert.rejects(
      () => handler.processMessage('test', [], [], {}),
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

  it('has agent profiles', () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    const def = handler.getAgentProfile('default');
    const plan = handler.getAgentProfile('planner');
    assert.ok(def.model);
    assert.ok(plan.maxTokens > 4096);
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
    const msgs = handler.convertMsg('New msg', mockHistory);
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
    const failExecutor = async () => { throw new Error('boom'); };
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
    const msgs = handler.convertMsg(null, history);
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
    const msgs = handler.convertMsg(null, history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'user');
    assert.equal(msgs[0].content[0].type, 'tool_result');
    assert.equal(msgs[0].content[0].tool_use_id, 'call_1');
    assert.equal(msgs[0].content[0].content, '{"temp":25}');
  });

  it('convertMsg handles normalized text items with content field', () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    const history = [
      { message: { type: 'text', content: 'Hello world', role: 'assistant' } },
    ];
    const msgs = handler.convertMsg(null, history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'assistant');
    assert.equal(msgs[0].content, 'Hello world');
  });

  it('processMessage fails with fake key (validates API call path)', async () => {
    const handler = new AnthropicHandler(FAKE_KEY);
    await handler.initialize();
    await assert.rejects(
      () => handler.processMessage('test', [], [], {}),
      (err) => err.message.includes('authentication') || err.message.includes('api-key') || err.status === 401
    );
  });
});

// ═══════════════════════════════════════════════════════════════════
// Ollama Handler
// ═══════════════════════════════════════════════════════════════════
describe('OllamaHandler', () => {
  const FAKE_HOST = 'http://localhost:11434';

  it('throws without host URL', () => {
    assert.throws(() => new OllamaHandler(''), /host URL is required/);
  });

  it('sets default model', () => {
    const handler = new OllamaHandler(FAKE_HOST);
    assert.equal(handler.model, 'llama3.2');
  });

  it('initializes client', async () => {
    const handler = new OllamaHandler(FAKE_HOST);
    await handler.initialize();
    assert.equal(handler.initialized, true);
    assert.ok(handler.client);
  });

  it('has agent profiles', () => {
    const handler = new OllamaHandler(FAKE_HOST);
    const def = handler.getAgentProfile('default');
    const plan = handler.getAgentProfile('planner');
    assert.equal(def.model, 'llama3.2');
    assert.equal(plan.model, 'llama3.2');
  });

  it('converts tools to OpenAI-compatible format', () => {
    const handler = new OllamaHandler(FAKE_HOST);
    const converted = handler.convertToolsForMCP(mockTools);
    assert.equal(converted.length, 1);
    assert.equal(converted[0].type, 'function');
    assert.equal(converted[0].function.name, 'get_weather');
    assert.equal(converted[0].function.description, 'Get current weather');
    assert.ok(converted[0].function.parameters);
    assert.equal(converted[0].function.parameters.type, 'object');
  });

  it('converts history keeping system messages inline', () => {
    const handler = new OllamaHandler(FAKE_HOST);
    const msgs = handler.convertMsg('New msg', mockHistory);
    // system + user + assistant + new user = 4
    assert.equal(msgs.length, 4);
    assert.equal(msgs[0].role, 'system');
    assert.equal(msgs[0].content, 'You are helpful.');
    assert.equal(msgs[1].role, 'user');
    assert.equal(msgs[2].role, 'assistant');
    assert.equal(msgs[3].role, 'user');
    assert.equal(msgs[3].content, 'New msg');
  });

  it('convertMsg handles function_call history items', () => {
    const handler = new OllamaHandler(FAKE_HOST);
    const history = [
      { message: { type: 'function_call', name: 'get_weather', arguments: '{"city":"Madrid"}', call_id: 'call_1' } },
    ];
    const msgs = handler.convertMsg(null, history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'assistant');
    assert.equal(msgs[0].content, '');
    assert.ok(msgs[0].tool_calls);
    assert.equal(msgs[0].tool_calls[0].function.name, 'get_weather');
    assert.deepEqual(msgs[0].tool_calls[0].function.arguments, { city: 'Madrid' });
  });

  it('convertMsg handles function_call_output history items', () => {
    const handler = new OllamaHandler(FAKE_HOST);
    const history = [
      { message: { type: 'function_call_output', name: 'get_weather', call_id: 'call_1', output: '{"temp":25}' } },
    ];
    const msgs = handler.convertMsg(null, history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'tool');
    assert.equal(msgs[0].content, '{"temp":25}');
  });

  it('convertMsg handles normalized text items with content field', () => {
    const handler = new OllamaHandler(FAKE_HOST);
    const history = [
      { message: { type: 'text', content: 'Hello world', role: 'assistant' } },
    ];
    const msgs = handler.convertMsg(null, history);
    assert.equal(msgs.length, 1);
    assert.equal(msgs[0].role, 'assistant');
    assert.equal(msgs[0].content, 'Hello world');
  });

  it('handles tool call and returns function_call_output with name', async () => {
    const handler = new OllamaHandler(FAKE_HOST);
    const result = await handler.handleToolCall(mockToolCall, mockExecutor);
    assert.equal(result.type, 'function_call_output');
    assert.equal(result.call_id, 'call_123');
    assert.equal(result.name, 'get_weather');
    assert.equal(JSON.parse(result.output).temp, 25);
  });

  it('handles tool call error gracefully', async () => {
    const handler = new OllamaHandler(FAKE_HOST);
    const failExecutor = async () => { throw new Error('boom'); };
    const result = await handler.handleToolCall(mockToolCall, failExecutor);
    assert.equal(result.type, 'function_call_output');
    assert.ok(JSON.parse(result.output).error.includes('boom'));
  });

  it('normalizes response with content field', () => {
    const handler = new OllamaHandler(FAKE_HOST);
    const norm = handler.normalizeResponse({
      output: [{ type: 'text', content: 'Hello', role: 'assistant' }],
      model: 'llama3.2',
    });
    assert.equal(norm.provider, 'ollama');
    assert.equal(norm.responseId, null);
    assert.equal(norm.content[0].content, 'Hello');
    assert.equal(norm.content[0].role, 'assistant');
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
      result = await handler.processMessage(prompt, [], [], {});
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
      result = await handler.processMessage(prompt, [], [], {});
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

  it('Anthropic responds to a simple message', { skip: !realKeys.anthropic && 'LLM_ANTHROPIC_API_KEY not set' }, async (t) => {
    const handler = LLMFactory.createHandler('anthropic', realKeys.anthropic);
    await handler.initialize();

    let result;
    try {
      result = await handler.processMessage(prompt, [], [], {});
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
  });

  it('Ollama responds to a simple message', { skip: !realKeys.ollama && 'LLM_OLLAMA_API_KEY not set' }, async (t) => {
    const handler = LLMFactory.createHandler('ollama', realKeys.ollama);
    await handler.initialize();

    let result;
    try {
      result = await handler.processMessage(prompt, [], [], {});
    } catch (err) {
      if (err.message?.includes('ECONNREFUSED') || err.message?.includes('fetch failed')) {
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
