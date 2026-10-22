import { describe, it, beforeEach, afterEach } from 'node:test';
import assert from 'node:assert/strict';
import { AuditUnavailableError, deps, EveOrchestrator } from '../models/chat/eveOrchestrator.js';
import { normalizeInputRequest } from '../models/chat/eveEvents.js';
import { eventBus, EVENTS } from '../common/eventBus.js';

const CHAT = 'chat_orch';

const request = normalizeInputRequest({
  kind: 'tool-approval',
  requestId: 'aitxt-1',
  prompt: 'Approve Multiuav gcs: Start mission?',
  options: [
    { id: 'approve', label: 'Approve' },
    { id: 'cancel', label: 'Cancel' },
  ],
  action: { kind: 'tool-call', callId: 'call-1', toolName: 'multiuav-gcs__start_mission', input: { deviceId: 2 } },
});

/** Swaps the orchestrator's collaborators for spies that keep one ordered log of what happened. */
const original = { ...deps };
let log;
let failAudit;
let failPost;
let running;

beforeEach(() => {
  log = [];
  failAudit = false;
  failPost = false;
  // The orchestrator forgets an answered request when the reader stops; a test that needs the
  // reader "still running" turns this on itself and stops it again.
  running = false;
  deps.store = {
    load: async () => ({ agent: 'assistant', sessionId: 'wrun_1', cursor: 0, turn: { pending: [request] } }),
  };
  deps.audit = {
    record: async (row) => {
      if (failAudit) throw new Error('database is locked');
      log.push(['audit.record', row.kind, row.source, row.outcome]);
      return true;
    },
    recordSafe: async (row) => {
      log.push(['audit.recordSafe', row.kind, row.source, row.outcome]);
      return true;
    },
  };
  deps.client = {
    sendInputResponses: async (agent, sessionId, responses) => {
      if (failPost) throw new Error('eve is down');
      log.push(['post', sessionId, responses]);
    },
  };
  deps.relay = { ensure: async () => log.push(['relay.ensure']), isRunning: () => running };
});

afterEach(() => Object.assign(deps, original));

describe('EveOrchestrator.respondToInput', () => {
  it('writes the decision to the audit BEFORE sending it to eve, and sends exactly what was answered', async () => {
    const result = await EveOrchestrator.respondToInput(CHAT, [{ requestId: 'aitxt-1', optionId: 'approve' }]);

    assert.deepEqual(result, { sent: ['aitxt-1'], stale: [] });
    assert.deepEqual(
      log.map((entry) => entry[0]),
      ['audit.record', 'post', 'relay.ensure']
    );
    assert.deepEqual(log[0], ['audit.record', 'decided', 'operator', 'approved']);
    assert.deepEqual(log[1][2], [{ requestId: 'aitxt-1', optionId: 'approve' }]);
  });

  it('records any other option as a denial and passes the reason on', async () => {
    await EveOrchestrator.respondToInput(CHAT, [{ requestId: 'aitxt-1', optionId: 'cancel', text: 'viento' }]);

    assert.deepEqual(log[0], ['audit.record', 'decided', 'operator', 'denied']);
    assert.deepEqual(log[1][2], [{ requestId: 'aitxt-1', optionId: 'cancel', text: 'viento' }]);
  });

  it('FAILS CLOSED: if the decision cannot be audited, nothing is sent to eve and the operator is told', async () => {
    failAudit = true;
    const told = [];
    const listener = (event) => told.push(event);
    eventBus.on(EVENTS.CHAT_TOOL_APPROVAL_FAILED, listener);

    try {
      await assert.rejects(
        () => EveOrchestrator.respondToInput(CHAT, [{ requestId: 'aitxt-1', optionId: 'approve' }]),
        AuditUnavailableError
      );
    } finally {
      eventBus.off(EVENTS.CHAT_TOOL_APPROVAL_FAILED, listener);
    }

    assert.deepEqual(log, [], 'no POST, no reader: the approval stays open and the operator can try again');
    assert.equal(told.length, 1);
    assert.equal(told[0].chatId, CHAT);
    assert.equal(told[0].failures[0].requestId, 'aitxt-1', 'the card of THIS request must show the error and stay');
    assert.match(told[0].failures[0].message, /nothing was sent/);
  });

  it('can be retried after the audit failure, because the request was not marked as answered', async () => {
    failAudit = true;
    await assert.rejects(() => EveOrchestrator.respondToInput(CHAT, [{ requestId: 'aitxt-1', optionId: 'approve' }]));

    failAudit = false;
    const result = await EveOrchestrator.respondToInput(CHAT, [{ requestId: 'aitxt-1', optionId: 'approve' }]);

    assert.deepEqual(result.sent, ['aitxt-1']);
  });

  it('leaves a row saying the decision was not delivered when eve cannot be reached', async () => {
    failPost = true;

    await assert.rejects(
      () => EveOrchestrator.respondToInput(CHAT, [{ requestId: 'aitxt-1', optionId: 'approve' }]),
      /eve is down/
    );

    assert.deepEqual(
      log.map((entry) => entry.slice(0, 4)),
      [
        ['audit.record', 'decided', 'operator', 'approved'],
        ['audit.recordSafe', 'decided', 'system', 'invalid'],
      ]
    );
    assert.ok(!log.some((entry) => entry[0] === 'relay.ensure'));
  });

  it('refuses an answer to a request it does not have open, and an option the request never offered', async () => {
    const result = await EveOrchestrator.respondToInput(CHAT, [
      { requestId: 'nope', optionId: 'approve' },
      { requestId: 'aitxt-1', optionId: 'approve-everything' },
    ]);

    assert.deepEqual(result.sent, []);
    assert.deepEqual(
      result.stale.map((s) => s.reason),
      ['stale', 'invalid_option']
    );
    assert.deepEqual(log, []);
  });

  it('does not send the same answer twice while eve has not confirmed the first (double click)', async () => {
    running = true; // the reader is still waiting for eve to confirm
    await EveOrchestrator.respondToInput(CHAT, [{ requestId: 'aitxt-1', optionId: 'approve' }]);
    const second = await EveOrchestrator.respondToInput(CHAT, [{ requestId: 'aitxt-1', optionId: 'approve' }]);

    assert.deepEqual(second.stale, [{ requestId: 'aitxt-1', reason: 'already_answered' }]);
    assert.equal(log.filter((entry) => entry[0] === 'post').length, 1);

    // The reader stops: the orchestrator forgets the answered request, so it cannot leak into other tests.
    running = false;
    EveOrchestrator._startReader(CHAT);
    await new Promise((resolve) => setImmediate(resolve));
  });

  it('does not audit an answer that is not about a tool (a session limit has no flight in it)', async () => {
    const limit = normalizeInputRequest({
      kind: 'session-limit',
      requestId: 'lim-1',
      options: [
        { id: 'continue', label: 'Continue' },
        { id: 'stop', label: 'Stop' },
      ],
    });
    deps.store = { load: async () => ({ agent: 'assistant', sessionId: 'wrun_1', turn: { pending: [limit] } }) };

    await EveOrchestrator.respondToInput(CHAT, [{ requestId: 'lim-1', optionId: 'continue' }]);

    assert.deepEqual(
      log.map((entry) => entry[0]),
      ['post', 'relay.ensure']
    );
  });
});

describe('EveOrchestrator.getPendingApprovals', () => {
  it('serves the approvals still open from the stored turn, so a reload does not lose them', async () => {
    const pending = await EveOrchestrator.getPendingApprovals(CHAT);

    assert.equal(pending.length, 1);
    assert.equal(pending[0].requestId, 'aitxt-1');
    assert.equal(pending[0].chatId, CHAT);
    assert.deepEqual(
      pending[0].options.map((o) => o.id),
      ['approve', 'cancel']
    );
  });

  it('is empty when the chat has no turn', async () => {
    deps.store = { load: async () => ({ agent: 'assistant', sessionId: 'wrun_1', turn: null }) };
    assert.deepEqual(await EveOrchestrator.getPendingApprovals(CHAT), []);
  });
});
