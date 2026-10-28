import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { MessageOrchestrator, legacyDeps } from '../models/chat/chat.js';
import { ChatHistoryManager } from '../models/chat/chatHistoryManager.js';
import { TurnContext } from '../models/chat/turnContext.js';
import { ToolApprovalStore } from '../models/chat/toolApprovalStore.js';
import { eventBus, EVENTS } from '../common/eventBus.js';

const deferred = () => {
  let resolve;
  const promise = new Promise((r) => {
    resolve = r;
  });
  return { promise, resolve };
};
const tool = (id) => ({ type: 'function_call', call_id: id, name: 'read_status', arguments: '{}' });
const reply = (output) => ({ output, raw: {}, responseId: null });

function setup(t) {
  const previous = { ...legacyDeps };
  t.after(() => Object.assign(legacyDeps, previous));
  const history = [];
  const handler = {
    processMessage: async () => reply([{ type: 'text', role: 'assistant', content: 'done' }]),
    normalizeResponse: (result) => result,
    getProviderName: () => 'test',
    handleSessionError: async () => {},
    handleToolCall: async (call, execute) => ({
      type: 'function_call_output',
      call_id: call.call_id,
      output: JSON.stringify(await execute(call.name, {})),
    }),
  };
  Object.assign(legacyDeps, { llmHandler: handler, mcpClient: { executeTool: async () => ({ ok: true }) } });
  t.mock.method(TurnContext, 'build', async () => ({ agent: { name: 'test' }, tools: [] }));
  t.mock.method(ChatHistoryManager, 'loadHistory', async () => [...history]);
  t.mock.method(ChatHistoryManager, 'addMessage', async (chatId, from, message) => {
    const item = { chatId, from, message };
    history.push(item);
    return item;
  });
  t.mock.method(ChatHistoryManager, 'recordUsage', async () => {});
  t.mock.method(ChatHistoryManager, 'clearSession', async () => {});
  t.mock.method(ToolApprovalStore, 'listPending', async () => []);
  t.mock.method(ToolApprovalStore, 'cancelPending', async () => 0);
  t.mock.method(ToolApprovalStore, 'cancelUnexecuted', async () => 0);
  t.mock.method(MessageOrchestrator, '_parkForApproval', async () => false);
  return { handler, history };
}

describe('legacy chat cancellation', () => {
  it('aborts the provider, releases busy and accepts a subsequent message without an error bubble', async (t) => {
    const { handler, history } = setup(t);
    const started = deferred();
    const busy = [];
    const onBusy = (event) => {
      if (event.chatId === 'cancel-provider') busy.push(event.busy);
    };
    eventBus.on(EVENTS.CHAT_BUSY, onBusy);
    t.after(() => eventBus.off(EVENTS.CHAT_BUSY, onBusy));
    handler.processMessage = async (_input, _tools, _history, { signal }) => {
      started.resolve(signal);
      await new Promise((_resolve, reject) =>
        signal.addEventListener('abort', () => reject(signal.reason), { once: true })
      );
    };
    const sending = MessageOrchestrator.processMessage('cancel-provider', 'hello');
    const signal = await started.promise;
    assert.deepEqual(await MessageOrchestrator.cancel('cancel-provider'), { cancelled: true });
    assert.equal(signal.aborted, true);
    assert.equal((await sending).cancelled, true);
    assert.equal(busy.at(-1), false);
    assert.equal(
      history.some(({ message }) => message.content?.startsWith('Error:')),
      false
    );
    handler.processMessage = async () => reply([{ type: 'text', content: 'next answer' }]);
    await MessageOrchestrator.processMessage('cancel-provider', 'again');
    assert.equal(history.at(-1).message.content, 'next answer');
    assert.deepEqual(await MessageOrchestrator.cancel('cancel-provider'), { cancelled: false });
  });

  it('suppresses a late provider reply when the provider ignores abort', async (t) => {
    const { handler, history } = setup(t);
    const started = deferred();
    const response = deferred();
    handler.processMessage = async () => {
      started.resolve();
      return response.promise;
    };
    const sending = MessageOrchestrator.processMessage('late-provider', 'hello');
    await started.promise;
    const stopping = MessageOrchestrator.cancel('late-provider');
    response.resolve(reply([tool('late-call')]));
    await Promise.all([sending, stopping]);
    assert.equal(
      history.some(({ message }) => message.call_id === 'late-call'),
      false
    );
  });

  it('stops a tool batch, closes every tool call and does not ask the model again', async (t) => {
    const { handler, history } = setup(t);
    const started = deferred();
    let requests = 0;
    let executions = 0;
    handler.processMessage = async () => {
      requests += 1;
      return reply([tool('one'), tool('two')]);
    };
    legacyDeps.mcpClient.executeTool = async (_name, _input, { signal }) => {
      executions += 1;
      started.resolve();
      await new Promise((_resolve, reject) =>
        signal.addEventListener('abort', () => reject(signal.reason), { once: true })
      );
    };
    await MessageOrchestrator.processMessage('cancel-tools', 'check');
    await started.promise;
    await MessageOrchestrator.cancel('cancel-tools');
    assert.equal(requests, 1);
    assert.equal(executions, 1);
    const outputs = history.filter(({ message }) => message.type === 'function_call_output');
    assert.deepEqual(
      outputs.map(({ message }) => message.call_id),
      ['one', 'two']
    );
    assert.ok(outputs.every(({ message }) => message.status === 'cancelled'));
  });

  it('holds the mutex through recursive tool iterations', async (t) => {
    const { handler } = setup(t);
    const thirdRequest = deferred();
    const response = deferred();
    let requests = 0;
    handler.processMessage = async () => {
      requests += 1;
      if (requests <= 2) return reply([tool(`call-${requests}`)]);
      thirdRequest.resolve();
      return response.promise;
    };
    await MessageOrchestrator.processMessage('loop-lock', 'first');
    await thirdRequest.promise;
    const queued = MessageOrchestrator.processMessage('loop-lock', 'second');
    await new Promise((resolve) => setImmediate(resolve));
    assert.equal(requests, 3);
    handler.processMessage = async () => reply([{ type: 'text', content: 'done' }]);
    response.resolve(reply([{ type: 'text', content: 'first done' }]));
    await queued;
  });

  it('cancels parked approvals without executing or resuming their turn', async (t) => {
    const { history } = setup(t);
    history.push({ message: tool('parked-call') });
    const row = { chatId: 'parked', turnId: 'turn', requestId: 'request', callId: 'parked-call', status: 'pending' };
    t.mock.method(ToolApprovalStore, 'listPending', async () => (row.status === 'pending' ? [row] : []));
    t.mock.method(ToolApprovalStore, 'cancelPending', async () => {
      row.status = 'cancelled';
      return 1;
    });
    t.mock.method(ToolApprovalStore, 'listPendingForTurn', async () => []);
    t.mock.method(ToolApprovalStore, 'listForTurn', async () => [row]);
    const execute = t.mock.method(MessageOrchestrator, 'executeToolCalls', async () => []);
    await MessageOrchestrator.cancel('parked');
    await MessageOrchestrator._resumeParkedTurn('parked', 'turn');
    assert.equal(execute.mock.callCount(), 0);
    assert.equal(history.at(-1).message.call_id, 'parked-call');
    assert.equal(history.at(-1).message.status, 'cancelled');
  });

  it('reports cleanup failures to the caller instead of claiming a successful stop', async (t) => {
    const { handler } = setup(t);
    const started = deferred();
    handler.processMessage = async (_input, _tools, _history, { signal }) => {
      started.resolve();
      await new Promise((_resolve, reject) =>
        signal.addEventListener('abort', () => reject(signal.reason), { once: true })
      );
    };
    t.mock.method(ToolApprovalStore, 'cancelPending', async () => {
      throw new Error('storage unavailable');
    });
    const sending = MessageOrchestrator.processMessage('cleanup-failure', 'hello');
    const rejected = assert.rejects(sending, /storage unavailable/);
    await started.promise;
    await assert.rejects(MessageOrchestrator.cancel('cleanup-failure'), /storage unavailable/);
    await rejected;
  });
});
