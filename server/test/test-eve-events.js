import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import {
  activityOf,
  initialTurnState,
  isReadComplete,
  normalizeInputRequest,
  reduceEvent,
  toAuditOutcome,
  TURN_PHASE,
} from '../models/chat/eveEvents.js';
import { eveEvent, loadEveFixture } from './helpers/eveFixtures.js';

/** Runs events through the reducer; returns the last state and every effect, in order. */
const run = (events, state = initialTurnState()) => {
  const effects = [];
  const phases = [];
  for (const event of events) {
    const result = reduceEvent(state, event);
    state = result.state;
    effects.push(...result.effects);
    phases.push(state.phase);
  }
  return { state, effects, phases };
};

const kinds = (effects) => effects.map((effect) => effect.kind);

describe('eve turn reducer — a turn with a subagent task (real capture)', () => {
  const events = loadEveFixture('subagent');
  const { state, effects, phases } = run(events);

  it('ends the turn on turn.completed and settles on session.waiting', () => {
    assert.equal(state.phase, TURN_PHASE.COMPLETED);
    assert.equal(state.settled, true);
    assert.equal(isReadComplete(state), true);
  });

  it('keeps the LAST message.completed as the answer, not the interim ones', () => {
    const texts = events.filter((e) => e.type === 'message.completed').map((e) => e.data.message);
    assert.ok(texts.length > 1, 'the capture must have interim messages for this to mean anything');
    const end = effects.find((effect) => effect.kind === 'turn_end');
    assert.equal(end.finalText, texts.at(-1));
    assert.notEqual(end.finalText, texts[0]);
  });

  it('reads turn.waiting on tasks as a turn still running, never as parked on a person', () => {
    assert.ok(!phases.includes(TURN_PHASE.AWAITING_INPUT));
    assert.ok(events.some((e) => e.type === 'turn.waiting' && e.data.on === 'tasks'));
  });

  it('projects the subagent call with the name eve gives it (no toolName on subagent calls)', () => {
    const call = effects.find((effect) => effect.kind === 'tool_call');
    assert.equal(call.name, 'planner_sandbox');
    assert.equal(call.isSubagent, true);
    assert.ok(call.callId);
  });

  it('turns the task into start/settle effects', () => {
    const tasks = effects.filter((effect) => effect.kind === 'task');
    assert.deepEqual(
      tasks.map((task) => task.status),
      ['started', 'completed']
    );
    assert.equal(tasks[0].name, 'planner_sandbox');
    assert.ok(tasks[1].output);
  });

  it('reports the planning label while the task is open, and none after', () => {
    const during = run(events.slice(0, events.findIndex((e) => e.type === 'task.started') + 1));
    assert.deepEqual(activityOf(during.state), { phase: 'planning', label: 'planner_sandbox' });
    assert.deepEqual(activityOf(state), { phase: 'idle', label: null });
  });

  it('ignores the text deltas', () => {
    const noise = events.filter((e) => ['message.appended', 'action.input.appended'].includes(e.type));
    assert.ok(noise.length > 0);
    assert.deepEqual(run(noise, initialTurnState({ phase: TURN_PHASE.RUNNING })).effects, []);
  });
});

describe('eve turn reducer — a tool approval (synthetic, built from the reported shapes)', () => {
  const pending = loadEveFixture('approval-pending.synthetic');
  const denied = loadEveFixture('approval-denied.tail.synthetic');
  const steered = loadEveFixture('approval-steered.tail.synthetic');

  it('PARKS on turn.waiting {on:"input"} instead of waiting for a session.waiting that never comes', () => {
    const { state, effects } = run(pending);
    assert.equal(state.phase, TURN_PHASE.AWAITING_INPUT);
    assert.equal(isReadComplete(state), true, 'the reader must stop here, not hang');
    assert.ok(!kinds(effects).includes('turn_end'));
    assert.equal(state.pending.length, 1);
  });

  it('shows the approval with eve options and the exact input, and no expiry or hash', () => {
    const { effects } = run(pending);
    const requested = effects.find((effect) => effect.kind === 'input_requested');
    const [request] = requested.requests;
    assert.equal(request.requestId, 'aitxt-SYN1');
    assert.equal(request.kind, 'tool-approval');
    assert.equal(request.source, 'eve');
    assert.deepEqual(
      request.options.map((o) => o.id),
      ['approve', 'cancel']
    );
    assert.equal(request.toolName, 'multiuav-gcs__start_mission');
    assert.deepEqual(request.action.input, { deviceId: 2 });
    assert.equal(request.expiresAt, undefined);
    assert.equal(request.action.inputHash, undefined);
  });

  it('projects the gated call as a tool call, not as a result', () => {
    const { effects } = run(pending);
    assert.deepEqual(
      effects.filter((e) => e.kind === 'tool_call').map((e) => e.name),
      ['multiuav-gcs__start_mission']
    );
    assert.ok(!kinds(effects).includes('tool_result'));
  });

  it('resumes after the answer, records the denial and finishes the turn', () => {
    const parked = run(pending).state;
    const { state, effects, phases } = run(denied, parked);

    assert.equal(phases[0], TURN_PHASE.RUNNING, 'resolving the last open request unparks the turn');
    const resolved = effects.find((effect) => effect.kind === 'input_resolved');
    assert.equal(resolved.resolutions[0].outcome, 'denied');
    assert.equal(resolved.resolutions[0].response.text, 'No ejecutar');
    assert.equal(resolved.resolutions[0].request.kind, 'tool-approval');

    const result = effects.find((effect) => effect.kind === 'tool_result');
    assert.equal(result.status, 'rejected');
    assert.equal(result.isError, true);
    assert.equal(result.requestId, 'aitxt-SYN1', 'the result is tied back to the decision on it');

    assert.equal(state.phase, TURN_PHASE.COMPLETED);
    assert.deepEqual(state.pending, []);
    assert.equal(effects.find((e) => e.kind === 'turn_end').finalText, 'No inicié la misión: la rechazaste.');
  });

  it('reads free text sent during an approval as `ignored`, which is not an approval', () => {
    const { effects } = run(steered, run(pending).state);
    const resolved = effects.find((effect) => effect.kind === 'input_resolved');
    assert.equal(resolved.resolutions[0].outcome, 'ignored');
    assert.equal(toAuditOutcome('ignored'), 'ignored');
    assert.equal(toAuditOutcome('answered'), null, '`answered` says nothing about an approval');
  });

  it('does not park on turn.waiting {on:"input"} when nothing is open (a replayed event)', () => {
    const { state } = run([eveEvent('turn.started'), eveEvent('turn.waiting', { on: 'input' })]);
    assert.equal(state.phase, TURN_PHASE.RUNNING);
  });

  it('closes the open approvals when the turn is cancelled, so no card outlives it', () => {
    const { effects, state } = run([eveEvent('turn.cancelled')], run(pending).state);
    const resolved = effects.find((effect) => effect.kind === 'input_resolved');
    assert.equal(resolved.derived, true);
    assert.equal(resolved.resolutions[0].requestId, 'aitxt-SYN1');
    assert.equal(resolved.resolutions[0].outcome, 'cancelled');
    assert.equal(state.phase, TURN_PHASE.CANCELLED);
    assert.deepEqual(state.pending, []);
  });
});

describe('eve turn reducer — edges', () => {
  it('ignores a session.waiting that belongs to a turn that already ended', () => {
    const { state } = run([eveEvent('session.waiting', { wait: 'next-user-message' })]);
    assert.equal(state.phase, TURN_PHASE.IDLE);
    assert.equal(isReadComplete(state), false, 'it must not settle the turn that has not started yet');
  });

  it('settles on session.waiting if no turn.* event closed the turn, so the chat cannot hang', () => {
    const { state, effects } = run([
      eveEvent('turn.started'),
      eveEvent('message.completed', { message: 'hola', finishReason: 'stop' }),
      eveEvent('session.waiting'),
    ]);
    assert.equal(isReadComplete(state), true);
    assert.equal(effects.find((e) => e.kind === 'turn_end').finalText, 'hola');
  });

  it('starts a new turn from a clean message, so a silent turn does not repeat the last answer', () => {
    const first = run([
      eveEvent('turn.started'),
      eveEvent('message.completed', { message: 'viejo' }),
      eveEvent('turn.completed'),
    ]);
    const second = run([eveEvent('turn.started'), eveEvent('turn.completed')], first.state);
    assert.equal(second.effects.find((e) => e.kind === 'turn_end').finalText, '');
  });

  it('fails the turn on turn.failed and carries the reason', () => {
    const { state, effects } = run([eveEvent('turn.started'), eveEvent('turn.failed', { error: { message: 'boom' } })]);
    assert.equal(state.phase, TURN_PHASE.FAILED);
    const end = effects.find((e) => e.kind === 'turn_end');
    assert.equal(end.error, 'boom');
    assert.equal(end.finalText, '');
  });

  it('does not end the turn on turn.failed with terminal:false (eve retries it)', () => {
    const { state } = run([eveEvent('turn.started'), eveEvent('turn.failed', { terminal: false })]);
    assert.equal(state.phase, TURN_PHASE.RUNNING);
  });

  it('hides skill loads: the agent reading its own manual is not for the operator', () => {
    const { effects } = run([
      eveEvent('actions.requested', { actions: [{ callId: 'c1', kind: 'load-skill', name: 'mission-planning' }] }),
      eveEvent('action.result', {
        status: 'completed',
        result: { callId: 'c1', kind: 'tool-result', output: 'skill text' },
      }),
    ]);
    assert.deepEqual(effects, []);
  });

  it('names a plain tool call by toolName', () => {
    const { effects } = run([
      eveEvent('actions.requested', {
        actions: [{ callId: 'c1', kind: 'tool-call', toolName: 'multiuav-gcs__get_devices', input: {} }],
      }),
    ]);
    assert.equal(effects[0].name, 'multiuav-gcs__get_devices');
    assert.equal(effects[0].isSubagent, false);
  });

  it('an approval that is not a tool (session limit) has no action and still gets its options', () => {
    const request = normalizeInputRequest({
      kind: 'session-limit',
      requestId: 'r1',
      prompt: 'Budget reached',
      options: [{ id: 'continue', label: 'Continue' }, { id: 'stop' }],
    });
    assert.equal(request.action, null);
    assert.equal(request.toolName, null);
    assert.deepEqual(
      request.options.map((o) => o.label),
      ['Continue', 'stop']
    );
  });

  it('does not duplicate an approval that is requested twice', () => {
    const event = eveEvent('input.requested', {
      requests: [
        {
          kind: 'tool-approval',
          requestId: 'r1',
          options: [{ id: 'approve' }],
          action: { callId: 'c', toolName: 't', input: {} },
        },
      ],
    });
    const { state, effects } = run([event, event]);
    assert.equal(state.pending.length, 1);
    assert.equal(effects.length, 1);
  });
});
