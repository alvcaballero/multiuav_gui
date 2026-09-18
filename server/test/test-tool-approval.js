import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import {
  canonicalizeInput,
  computeInputHash,
  freezeToolCall,
  isExpired,
  assertApprovedInputMatches,
  ToolApprovalMismatchError,
  ToolApprovalExpiredError,
} from '../models/chat/toolApproval.js';

describe('canonicalizeInput', () => {
  it('is independent of key insertion order', () => {
    const a = { uav: 3, missionId: 'A', altitude: 40 };
    const b = { altitude: 40, missionId: 'A', uav: 3 };

    assert.equal(canonicalizeInput(a), canonicalizeInput(b));
  });

  it('sorts nested object keys too', () => {
    const a = { outer: { z: 1, a: 2 } };
    const b = { outer: { a: 2, z: 1 } };

    assert.equal(canonicalizeInput(a), canonicalizeInput(b));
  });

  it('preserves array order, because waypoint order is the mission', () => {
    const forward = canonicalizeInput({ wp: [[0, 0, 10], [50, 0, 20]] });
    const reversed = canonicalizeInput({ wp: [[50, 0, 20], [0, 0, 10]] });

    assert.notEqual(forward, reversed);
  });

  it('drops undefined properties, matching what the tool actually receives', () => {
    assert.equal(canonicalizeInput({ a: 1, b: undefined }), canonicalizeInput({ a: 1 }));
  });

  it('writes undefined array holes as null, matching JSON.stringify', () => {
    assert.equal(canonicalizeInput([1, undefined, 3]), '[1,null,3]');
  });

  it('distinguishes null from absent', () => {
    assert.notEqual(canonicalizeInput({ altitude: null }), canonicalizeInput({}));
  });

  it('distinguishes a number from its string form', () => {
    assert.notEqual(canonicalizeInput({ altitude: 40 }), canonicalizeInput({ altitude: '40' }));
  });

  it('throws on NaN instead of silently writing null', () => {
    assert.throws(() => canonicalizeInput({ altitude: NaN }), TypeError);
  });

  it('throws on Infinity instead of silently writing null', () => {
    assert.throws(() => canonicalizeInput({ altitude: Infinity }), TypeError);
  });

  it('throws on a function, which cannot be part of a tool input', () => {
    assert.throws(() => canonicalizeInput({ cb: () => {} }), TypeError);
  });
});

describe('computeInputHash', () => {
  it('produces the same hash for the same logical input', () => {
    assert.equal(
      computeInputHash({ uav: 3, missionId: 'A', altitude: 40 }),
      computeInputHash({ altitude: 40, missionId: 'A', uav: 3 })
    );
  });

  it('changes when any value changes', () => {
    assert.notEqual(computeInputHash({ altitude: 40 }), computeInputHash({ altitude: 120 }));
  });

  it('is prefixed so the algorithm is on the wire', () => {
    assert.match(computeInputHash({ a: 1 }), /^sha256:[0-9a-f]{64}$/);
  });
});

describe('freezeToolCall', () => {
  it('carries the input, its hash and an expiry', () => {
    const frozen = freezeToolCall({
      callId: 'call_1',
      toolName: 'start_mission',
      input: { uav: 3, missionId: 'A', altitude: 40 },
    });

    assert.equal(frozen.callId, 'call_1');
    assert.equal(frozen.toolName, 'start_mission');
    assert.deepEqual(frozen.input, { uav: 3, missionId: 'A', altitude: 40 });
    assert.equal(frozen.inputHash, computeInputHash(frozen.input));
    assert.ok(frozen.requestId);
    assert.ok(new Date(frozen.expiresAt) > new Date(frozen.frozenAt));
  });

  it('gives every freeze its own requestId', () => {
    const args = { callId: 'c', toolName: 'start_mission', input: { a: 1 } };

    assert.notEqual(freezeToolCall(args).requestId, freezeToolCall(args).requestId);
  });
});

describe('assertApprovedInputMatches', () => {
  const frozen = () =>
    freezeToolCall({
      callId: 'call_1',
      toolName: 'start_mission',
      input: { uav: 3, missionId: 'A', altitude: 40 },
      ttlMs: 60000,
    });

  it('passes when the input is byte-identical', () => {
    assert.equal(assertApprovedInputMatches(frozen(), { uav: 3, missionId: 'A', altitude: 40 }), true);
  });

  it('passes when only key order differs', () => {
    assert.equal(assertApprovedInputMatches(frozen(), { altitude: 40, uav: 3, missionId: 'A' }), true);
  });

  // The scenario this whole module exists for: operator approves 40 m, the
  // resumed model re-derives 120 m.
  it('rejects a changed altitude', () => {
    assert.throws(
      () => assertApprovedInputMatches(frozen(), { uav: 3, missionId: 'A', altitude: 120 }),
      ToolApprovalMismatchError
    );
  });

  it('rejects a changed target UAV', () => {
    assert.throws(
      () => assertApprovedInputMatches(frozen(), { uav: 4, missionId: 'A', altitude: 40 }),
      ToolApprovalMismatchError
    );
  });

  it('rejects an extra argument the operator never saw', () => {
    assert.throws(
      () => assertApprovedInputMatches(frozen(), { uav: 3, missionId: 'A', altitude: 40, force: true }),
      ToolApprovalMismatchError
    );
  });

  it('rejects a dropped argument', () => {
    assert.throws(() => assertApprovedInputMatches(frozen(), { uav: 3, missionId: 'A' }), ToolApprovalMismatchError);
  });

  it('rejects once the approval has expired, even with a matching input', () => {
    const stale = freezeToolCall({
      callId: 'call_1',
      toolName: 'start_mission',
      input: { uav: 3 },
      ttlMs: 1,
    });

    assert.throws(
      () => assertApprovedInputMatches(stale, { uav: 3 }, Date.now() + 5000),
      ToolApprovalExpiredError
    );
  });

  it('reports both hashes so a mismatch is auditable', () => {
    const approved = frozen();
    try {
      assertApprovedInputMatches(approved, { uav: 3, missionId: 'A', altitude: 120 });
      assert.fail('should have thrown');
    } catch (error) {
      assert.equal(error.code, 'tool_approval_input_mismatch');
      assert.equal(error.approvedHash, approved.inputHash);
      assert.equal(error.actualHash, computeInputHash({ uav: 3, missionId: 'A', altitude: 120 }));
    }
  });
});

describe('isExpired', () => {
  it('is false inside the window and true past it', () => {
    const frozen = freezeToolCall({ callId: 'c', toolName: 'start_mission', input: {}, ttlMs: 1000 });

    assert.equal(isExpired(frozen, Date.now()), false);
    assert.equal(isExpired(frozen, Date.now() + 2000), true);
  });
});
