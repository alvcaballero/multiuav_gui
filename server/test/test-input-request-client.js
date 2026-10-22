import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { toInputRequest } from '../models/chat/toolApprovalStore.js';
import { normalizeInputRequest } from '../models/chat/eveEvents.js';
import {
  activityText,
  displayToolName,
  formatCountdown,
  isFlightApproval,
  isTimed,
  optionsOf,
  requestTitle,
  secondsLeft,
  splitOptions,
} from '../../client/src/common/util/inputRequest.js';

// The card renders a request without knowing which engine produced it. The client has no test
// runner, so the logic that decides what the card shows lives in a pure module and is checked
// here against the REAL output of both servers (`toInputRequest` and `normalizeInputRequest`).

const legacyRow = {
  requestId: 'req-legacy',
  chatId: 'chat-1',
  callId: 'call-1',
  toolName: 'start_mission',
  input: { uav: 3, altitude: 40 },
  inputHash: 'sha256:' + 'a'.repeat(64),
  status: 'pending',
  expiresAt: new Date(Date.now() + 120_000),
};

const eveRequest = normalizeInputRequest({
  kind: 'tool-approval',
  requestId: 'aitxt-1',
  options: [
    { id: 'approve', label: 'Approve' },
    { id: 'cancel', label: 'Cancel' },
  ],
  action: { kind: 'tool-call', callId: 'call-1', toolName: 'multiuav-gcs__start_mission', input: { deviceId: 2 } },
});

describe('what the approval card shows, for both engines', () => {
  it('a legacy request keeps its expiry and carries the options the card needs', () => {
    const request = toInputRequest(legacyRow);

    assert.equal(isTimed(request), true);
    assert.deepEqual(splitOptions(request).affirmative, { id: 'approve', label: 'Approve & execute' });
    assert.deepEqual(
      splitOptions(request).negatives.map((o) => o.id),
      ['deny']
    );
    assert.equal(isFlightApproval(request), true);
    assert.equal(requestTitle(request), 'start_mission');
  });

  it('an eve request has no expiry and answers with the options eve sent (cancel, not deny)', () => {
    assert.equal(isTimed(eveRequest), false);
    assert.equal(splitOptions(eveRequest).affirmative.id, 'approve');
    assert.deepEqual(
      splitOptions(eveRequest).negatives.map((o) => o.id),
      ['cancel']
    );
  });

  it('names an eve tool by the tool, not by the MCP server prefix', () => {
    assert.equal(displayToolName('multiuav-gcs__start_mission'), 'start_mission');
    assert.equal(requestTitle(eveRequest), 'start_mission');
    assert.equal(displayToolName('start_mission'), 'start_mission');
    assert.equal(displayToolName(null), null);
  });

  it('shows a session limit as a question, not as an aircraft alarm, with its own prompt', () => {
    const limit = normalizeInputRequest({
      kind: 'session-limit',
      requestId: 'lim',
      prompt: 'Token budget reached',
      options: [
        { id: 'continue', label: 'Continue' },
        { id: 'stop', label: 'Stop' },
      ],
    });

    assert.equal(isFlightApproval(limit), false);
    assert.equal(requestTitle(limit), 'Token budget reached');
    assert.equal(splitOptions(limit).affirmative.id, 'continue');
  });

  it('falls back to the legacy pair for a request that predates `options`', () => {
    assert.deepEqual(
      optionsOf({ requestId: 'old' }).map((o) => o.id),
      ['approve', 'deny']
    );
  });

  it('counts down only what expires, and never shows a negative time', () => {
    const now = 1_000_000;
    assert.equal(secondsLeft(new Date(now + 65_000), now), 65);
    assert.equal(secondsLeft(new Date(now - 5_000), now), 0);
    assert.equal(formatCountdown(65), '1:05');
    assert.equal(formatCountdown(0), '0:00');
  });

  it('says what a turn is doing, and says nothing when it is idle or waiting for the operator', () => {
    assert.equal(activityText({ phase: 'running', label: null }), 'Thinking…');
    assert.equal(
      activityText({ phase: 'planning', label: 'request_mission_plan' }),
      'Working on request_mission_plan…'
    );
    assert.equal(activityText({ phase: 'planning', label: 'planner_sandbox' }), 'Working on planner_sandbox…');
    assert.equal(activityText({ phase: 'awaiting_input', label: null }), null);
    assert.equal(activityText(null), null);
  });
});
