import { describe, it, afterEach } from 'node:test';
import assert from 'node:assert/strict';
import { once } from 'node:events';
import express from 'express';
import { OpenAIHandler } from '../models/chat/handlers/openaiHandler.js';
import { OpenAICompatibleHandler } from '../models/chat/handlers/openaiCompatibleHandler.js';
import { AnthropicHandler } from '../models/chat/handlers/antropicHandler.js';
import { GeminiHandler } from '../models/chat/handlers/geminiHandler.js';
import { MCPclient } from '../models/chat/mcpClient.js';
import { EveOrchestrator, deps } from '../models/chat/eveOrchestrator.js';
import { ChatHistoryManager } from '../models/chat/chatHistoryManager.js';
import { chatController } from '../controllers/chat.js';
import { MessageOrchestrator } from '../models/chat/chat.js';
import { chatRouter } from '../routes/chat.js';

const original = { ...deps };
afterEach(() => Object.assign(deps, original));
const waiting = () => {
  let resolve;
  const promise = new Promise((r) => {
    resolve = r;
  });
  return { promise, resolve };
};

describe('provider cancellation transport', () => {
  for (const [name, create, install, reply] of [
    [
      'OpenAI',
      () => new OpenAIHandler('test'),
      (h, fn) => {
        h.client = { responses: { create: fn } };
      },
      { output: [], status: 'completed' },
    ],
    [
      'compatible',
      () => new OpenAICompatibleHandler('test', 'model', 'system', { baseURL: 'http://example.invalid/v1' }),
      (h, fn) => {
        h.client = { chat: { completions: { create: fn } } };
      },
      { choices: [{ message: { content: 'ok' } }] },
    ],
    [
      'Anthropic',
      () => new AnthropicHandler('test'),
      (h, fn) => {
        h.client = { messages: { create: fn } };
      },
      { content: [], stop_reason: 'end_turn' },
    ],
    [
      'Gemini',
      () => new GeminiHandler('test'),
      (h, fn) => {
        h.client = { models: { generateContent: (params) => fn(params, { signal: params.config.abortSignal }) } };
      },
      { candidates: [] },
    ],
  ]) {
    it(`${name} forwards the turn signal and still supports ordinary requests`, async () => {
      const handler = create();
      const controller = new AbortController();
      const received = [];
      install(handler, async (_params, options) => {
        received.push(options.signal);
        return reply;
      });
      const input = { type: 'message', content: 'hello' };
      await handler.processMessage(input, [], [], { signal: controller.signal });
      await handler.processMessage(input);
      assert.deepEqual(received, [controller.signal, undefined]);
    });
  }

  it('preserves the abort signal through OpenAI session fallback', async () => {
    const handler = new OpenAIHandler('test');
    const controller = new AbortController();
    const signals = [];
    handler.client = {
      responses: {
        create: async (_params, { signal }) => {
          signals.push(signal);
          if (signals.length === 1) throw new Error('Conversation not found');
          return { output: [], status: 'completed' };
        },
      },
    };
    await handler.processMessage({ type: 'message', content: 'hello' }, [], [], {
      sessionId: 'conv_test',
      signal: controller.signal,
    });
    assert.deepEqual(signals, [controller.signal, controller.signal]);
  });

  it('passes cancellation to MCP without retrying the cancelled call', async () => {
    const client = new MCPclient();
    const controller = new AbortController();
    let calls = 0;
    client.isConnected = true;
    client.tools = [{ name: 'read_status' }];
    client.client = {
      callTool: async (_params, _schema, { signal }) => {
        calls += 1;
        assert.equal(signal, controller.signal);
        controller.abort();
        throw new Error('Connection closed');
      },
    };
    await assert.rejects(client.executeTool('read_status', {}, { signal: controller.signal }), { name: 'AbortError' });
    assert.equal(calls, 1);
  });
});

describe('Eve cancellation', () => {
  it('waits for first-session creation before cancelling and keeps the relay alive', async (t) => {
    const accepted = waiting();
    const started = waiting();
    let sessionId = null;
    const cancelled = [];
    let readers = 0;
    deps.store = { load: async () => ({ agent: 'assistant', sessionId }) };
    deps.client = {
      createSession: async () => {
        started.resolve();
        await accepted.promise;
        return { sessionId: 'session-new' };
      },
      cancelSession: async (...args) => {
        cancelled.push(args);
      },
    };
    deps.relay = {
      ensure: async () => {
        readers += 1;
      },
      isRunning: () => false,
    };
    t.mock.method(ChatHistoryManager, 'addMessage', async () => ({ from: 'user' }));
    t.mock.method(ChatHistoryManager, 'setSessionId', async (_chatId, id) => {
      sessionId = id;
    });
    t.mock.method(ChatHistoryManager, 'updateChat', async () => {});
    const sending = EveOrchestrator.processMessage('eve-new', 'hello');
    await started.promise;
    const stopping = EveOrchestrator.cancel('eve-new');
    assert.equal(cancelled.length, 0);
    accepted.resolve();
    await sending;
    assert.deepEqual(await stopping, { cancelled: true });
    assert.deepEqual(cancelled, [['assistant', 'session-new']]);
    assert.equal(readers, 2);
  });

  it('propagates a cancel failure so the UI can retry', async () => {
    deps.store = { load: async () => ({ agent: 'assistant', sessionId: 'session-existing' }) };
    deps.client = {
      cancelSession: async () => {
        throw new Error('eve unavailable');
      },
    };
    await assert.rejects(EveOrchestrator.cancel('eve-failure'), /eve unavailable/);
  });

  it('does not invent a session when there is no turn', async () => {
    deps.store = { load: async () => ({ sessionId: null }) };
    assert.deepEqual(await EveOrchestrator.cancel('eve-idle'), { cancelled: false });
  });
});

describe('chat cancellation HTTP endpoint', () => {
  it('routes the request to the chat engine and exposes cancellation errors', async (t) => {
    t.mock.method(chatController, '_resolveEngine', async () => 'eve');
    t.mock.method(EveOrchestrator, 'cancel', async (id) => ({ cancelled: id === 'chat-route' }));
    const app = express();
    app.use('/api/chat', chatRouter);
    const server = app.listen(0, '127.0.0.1');
    t.after(() => {
      server.closeAllConnections();
      server.close();
    });
    await once(server, 'listening');
    const url = `http://127.0.0.1:${server.address().port}/api/chat/chats/chat-route/cancel`;
    const response = await fetch(url, { method: 'POST' });
    assert.equal(response.status, 200);
    assert.deepEqual(await response.json(), { chatId: 'chat-route', cancelled: true });
    t.mock.method(chatController, '_resolveEngine', async () => 'legacy');
    t.mock.method(MessageOrchestrator, 'cancel', async () => ({ cancelled: false }));
    const legacy = await fetch(url, { method: 'POST' });
    assert.equal(legacy.status, 200);
    assert.deepEqual(await legacy.json(), { chatId: 'chat-route', cancelled: false });
    t.mock.method(chatController, '_resolveEngine', async () => 'eve');
    t.mock.method(EveOrchestrator, 'cancel', async () => {
      throw new Error('unavailable');
    });
    const failed = await fetch(url, { method: 'POST' });
    assert.equal(failed.status, 500);
    assert.deepEqual(await failed.json(), { error: 'unavailable' });
  });
});
