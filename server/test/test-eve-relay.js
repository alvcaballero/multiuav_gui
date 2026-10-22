import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { EveRelay } from '../models/chat/eveRelay.js';
import { EveSessionExpiredError } from '../models/chat/eveClient.js';
import { TURN_PHASE } from '../models/chat/eveEvents.js';
import { eveEvent, loadEveFixture } from './helpers/eveFixtures.js';

const CHAT = 'chat_test';

/**
 * A relay wired to fakes. `script.events` is what the "server" has; each read starts at its
 * `startIndex`, like a durable eve stream. `script.hang` keeps the stream OPEN after the last event
 * (what eve does while a turn waits for a person), `script.cutAt` closes it early once per entry.
 */
const makeRelay = ({ events = [], hang = false, cutAt = [], throwOnOpen = null, saved = {} } = {}) => {
  const script = { events, hang, cutAt: [...cutAt] };
  const calls = { opens: [], sleeps: [] };
  const out = { items: [], errors: [], busy: [], activity: [], requested: [], resolved: [], audit: [] };
  const stored = { agent: 'assistant', sessionId: 'wrun_1', cursor: 0, turn: null, ...saved };

  const client = {
    async *streamEvents(_agent, _sessionId, { startIndex, signal }) {
      calls.opens.push(startIndex);
      if (throwOnOpen) throw throwOnOpen;
      const limit = script.cutAt.length ? script.cutAt.shift() : Infinity;
      const slice = script.events.slice(startIndex, Math.min(script.events.length, limit));
      for (const event of slice) yield event;
      if (script.hang && limit === Infinity) {
        // Stay open until the reader aborts or leaves — the stream of a turn parked on a person.
        await new Promise((resolve) => {
          signal?.addEventListener('abort', resolve);
          setTimeout(resolve, 60_000).unref();
        });
      }
    },
  };

  const relay = new EveRelay({
    client,
    store: {
      load: async () => ({ ...stored }),
      save: async (_chatId, { cursor, turn }) => {
        stored.cursor = cursor;
        stored.turn = turn;
      },
    },
    items: {
      add: async (chatId, message) => {
        const item = { chatId, from: 'assistant', message };
        out.items.push(item);
        return item;
      },
    },
    emit: {
      assistantMessage: () => {},
      assistantError: (chatId, text) => out.errors.push(text),
      busy: (chatId, busy) => out.busy.push(busy),
      activity: (chatId, activity) => out.activity.push(activity),
      approvalRequested: (chatId, requests) => out.requested.push(...requests),
      approvalResolved: (chatId, resolutions) => out.resolved.push(...resolutions),
    },
    audit: { recordSafe: async (row) => out.audit.push(row) },
    sleep: async (ms) => calls.sleeps.push(ms),
  });

  return { relay, script, stored, out, calls };
};

const withTimeout = (promise, ms = 2000) =>
  Promise.race([
    promise,
    new Promise((_, reject) => setTimeout(() => reject(new Error(`timed out after ${ms}ms`)), ms).unref()),
  ]);

const messages = (out) => out.items.map((item) => item.message);

describe('EveRelay — a turn with a task (real capture)', () => {
  it('persists the tool call, its result and the final answer, and ends not busy', async () => {
    const events = loadEveFixture('subagent');
    const { relay, out, stored } = makeRelay({ events });

    await withTimeout(relay.ensure(CHAT));

    const types = messages(out).map((m) => m.type);
    assert.deepEqual(types, ['function_call', 'function_call_output', 'text']);
    assert.equal(messages(out)[0].name, 'planner_sandbox');
    assert.equal(typeof messages(out)[0].arguments, 'string', 'legacy items carry the arguments as a JSON string');
    assert.equal(messages(out)[2].content, events.filter((e) => e.type === 'message.completed').at(-1).data.message);

    assert.equal(out.busy.at(-1), false);
    assert.deepEqual(out.activity.map((a) => a.phase).includes('planning'), true);
    assert.equal(
      stored.cursor,
      events.length,
      'every event, including the closing session.waiting, is behind the cursor'
    );
    assert.equal(stored.turn.phase, TURN_PHASE.COMPLETED);
  });
});

describe('EveRelay — a tool approval', () => {
  const pending = loadEveFixture('approval-pending.synthetic');
  const denied = loadEveFixture('approval-denied.tail.synthetic');

  it('does NOT hang on a stream that stays open while a person decides (the original bug)', async () => {
    const { relay, out, stored } = makeRelay({ events: pending, hang: true });

    await withTimeout(relay.ensure(CHAT), 1000);

    assert.equal(out.requested.length, 1);
    assert.equal(out.requested[0].requestId, 'aitxt-SYN1');
    assert.equal(out.requested[0].chatId, CHAT, 'the client files the request under its chat');
    assert.equal(out.busy.at(-1), false, 'a parked turn must not keep the chat busy');
    assert.equal(out.activity.at(-1).phase, 'awaiting_input');
    assert.equal(stored.turn.phase, TURN_PHASE.AWAITING_INPUT);
    assert.equal(stored.turn.pending.length, 1, 'the open approval survives a restart in the stored state');
    assert.equal(stored.cursor, pending.length);
  });

  it('audits the request with the exact input the operator was shown', async () => {
    const { relay, out } = makeRelay({ events: pending, hang: true });
    await withTimeout(relay.ensure(CHAT), 1000);

    const row = out.audit.find((r) => r.kind === 'requested');
    assert.equal(row.engine, 'eve');
    assert.equal(row.requestId, 'aitxt-SYN1');
    assert.deepEqual(row.input, { deviceId: 2 });
    assert.equal(row.toolName, 'multiuav-gcs__start_mission');
  });

  it('resumes from the cursor after the answer and records the decision, the result and the answer', async () => {
    // First read stops where the turn parks, as it would live.
    const first = makeRelay({ events: pending, hang: true });
    await withTimeout(first.relay.ensure(CHAT), 1000);

    // The "server" now has the answer's events too; a new read picks up at the stored cursor.
    first.script.events = [...pending, ...denied];
    first.script.hang = false;
    await withTimeout(first.relay.ensure(CHAT), 1000);

    assert.deepEqual(first.calls.opens, [0, pending.length]);
    const types = messages(first.out).map((m) => m.type);
    assert.deepEqual(types, ['function_call', 'function_call_output', 'text']);
    assert.equal(messages(first.out)[2].content, 'No inicié la misión: la rechazaste.');

    const decided = first.out.audit.find((r) => r.kind === 'decided');
    assert.deepEqual(
      { source: decided.source, outcome: decided.outcome, text: decided.text },
      { source: 'eve', outcome: 'denied', text: 'No ejecutar' }
    );
    const executed = first.out.audit.find((r) => r.kind === 'executed');
    assert.equal(executed.requestId, 'aitxt-SYN1', 'the result is audited against the decision on it');
    assert.equal(executed.text, 'rejected');

    assert.deepEqual(first.out.resolved, [{ requestId: 'aitxt-SYN1', outcome: 'denied', responderPrincipalId: null }]);
    assert.equal(first.stored.cursor, pending.length + denied.length);
    assert.equal(first.stored.turn.phase, TURN_PHASE.COMPLETED);
    assert.deepEqual(first.stored.turn.pending, []);
  });
});

describe('EveRelay — the cursor', () => {
  it('does not replay the previous turn on the next one (the stale-answer bug)', async () => {
    const turnOne = loadEveFixture('subagent');
    const turnTwo = [
      eveEvent('message.received', { message: 'otra cosa' }),
      eveEvent('turn.started'),
      eveEvent('message.completed', { message: 'segunda respuesta', finishReason: 'stop' }),
      eveEvent('turn.completed'),
      eveEvent('session.waiting', { wait: 'next-user-message' }),
    ];
    const { relay, script, out, calls } = makeRelay({ events: turnOne });

    await withTimeout(relay.ensure(CHAT));
    const afterOne = out.items.length;

    script.events = [...turnOne, ...turnTwo];
    await withTimeout(relay.ensure(CHAT));

    assert.deepEqual(calls.opens, [0, turnOne.length]);
    const added = messages(out).slice(afterOne);
    assert.deepEqual(added, [{ role: 'assistant', type: 'text', content: 'segunda respuesta' }]);
  });

  it('skips an event it already saw (same meta.id) without projecting it twice', async () => {
    const call = eveEvent('actions.requested', {
      actions: [{ callId: 'c1', kind: 'tool-call', toolName: 't', input: {} }],
    });
    const events = [eveEvent('turn.started'), call, call, eveEvent('turn.completed'), eveEvent('session.waiting')];
    const { relay, out } = makeRelay({ events });

    await withTimeout(relay.ensure(CHAT));

    assert.equal(messages(out).filter((m) => m.type === 'function_call').length, 1);
  });

  it('reconnects when the stream closes mid-turn and continues where it stopped', async () => {
    const events = [
      eveEvent('turn.started'),
      eveEvent('actions.requested', { actions: [{ callId: 'c1', kind: 'tool-call', toolName: 't', input: {} }] }),
      eveEvent('message.completed', { message: 'listo', finishReason: 'stop' }),
      eveEvent('turn.completed'),
      eveEvent('session.waiting'),
    ];
    const { relay, out, calls } = makeRelay({ events, cutAt: [2] });

    await withTimeout(relay.ensure(CHAT));

    assert.deepEqual(calls.opens, [0, 2]);
    assert.deepEqual(calls.sleeps, [1000], 'a stream that made progress waits only the first backoff step');
    assert.deepEqual(
      messages(out).map((m) => m.type),
      ['function_call', 'text']
    );
  });
});

describe('EveRelay — when it cannot continue', () => {
  it('gives up with a visible error when the session history expired, without retrying', async () => {
    const { relay, out, calls, stored } = makeRelay({ throwOnOpen: new EveSessionExpiredError({ status: 410 }) });

    await withTimeout(relay.ensure(CHAT));

    assert.equal(calls.opens.length, 1);
    assert.match(out.errors[0], /no longer available/);
    assert.equal(stored.turn.phase, TURN_PHASE.FAILED);
  });

  it('gives up after the reconnect budget when eve stays unreachable', async () => {
    const { relay, out, calls } = makeRelay({ throwOnOpen: new TypeError('fetch failed') });

    await withTimeout(relay.ensure(CHAT));

    assert.equal(calls.opens.length, 6, 'the first try plus five reconnects');
    assert.deepEqual(calls.sleeps, [1000, 2000, 5000, 10000, 30000]);
    assert.match(out.errors[0], /Lost the connection/);
  });

  it('two ensure() calls share one reader', async () => {
    const { relay, calls } = makeRelay({ events: loadEveFixture('subagent') });
    const a = relay.ensure(CHAT);
    const b = relay.ensure(CHAT);
    assert.equal(a, b);
    await withTimeout(a);
    assert.equal(relay.isRunning(CHAT), false);
    assert.ok(calls.opens.length >= 1);
  });
});
