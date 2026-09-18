import { describe, it, before, after } from 'node:test';
import assert from 'node:assert/strict';
import sequelize from '../common/sequelize.js';
import { ToolApprovalStore, toInputRequest } from '../models/chat/toolApprovalStore.js';
import { freezeToolCall } from '../models/chat/toolApproval.js';

// Exercises the real SQL, because the properties under test ARE the atomicity of
// the UPDATE ... WHERE status='pending' — an in-memory fake would prove nothing.
const CHAT_ID = `test-approval-${process.pid}-${Date.now()}`;

const pending = async ({ toolName = 'start_mission', input = { uav: 3, altitude: 40 }, turnId = 'turn-1', ttlMs = 60000 } = {}) =>
  ToolApprovalStore.createPending({
    chatId: CHAT_ID,
    turnId,
    iteration: 1,
    frozen: freezeToolCall({ callId: `call-${Math.random()}`, toolName, input, ttlMs }),
    resumeContext: { allowedTools: null, toolCalls: [] },
  });

describe('ToolApprovalStore', () => {
  before(async () => {
    await sequelize.models.ChatToolApproval.sync();
  });

  after(async () => {
    await sequelize.models.ChatToolApproval.destroy({ where: { chatId: CHAT_ID } });
  });

  it('stores a pending request and lists it back', async () => {
    const row = await pending();

    const listed = await ToolApprovalStore.listPending(CHAT_ID);
    assert.ok(listed.some((r) => r.requestId === row.requestId));
    assert.equal(row.status, 'pending');
    assert.equal(row.responderPrincipalId, null);
  });

  it('approves once and records who did it', async () => {
    const row = await pending();

    const resolved = await ToolApprovalStore.resolve(row.requestId, {
      outcome: 'approved',
      responderPrincipalId: 'operator-1',
    });

    assert.equal(resolved.status, 'approved');
    assert.equal(resolved.responderPrincipalId, 'operator-1');
    assert.ok(resolved.resolvedAt);
  });

  // Two operators racing on the same request: first write wins, the loser is told.
  it('rejects a second answer to an already-resolved request', async () => {
    const row = await pending();

    const first = await ToolApprovalStore.resolve(row.requestId, {
      outcome: 'approved',
      responderPrincipalId: 'operator-1',
    });
    const second = await ToolApprovalStore.resolve(row.requestId, {
      outcome: 'denied',
      responderPrincipalId: 'operator-2',
    });

    assert.equal(first.status, 'approved');
    assert.equal(second, null);

    const stored = await ToolApprovalStore.getByRequestId(row.requestId);
    assert.equal(stored.status, 'approved');
    assert.equal(stored.responderPrincipalId, 'operator-1');
  });

  it('refuses to resolve an expired request', async () => {
    const row = await pending({ ttlMs: -1000 });

    assert.equal(await ToolApprovalStore.resolve(row.requestId, { outcome: 'approved' }), null);
  });

  it('leaves an expired request out of the pending list', async () => {
    const row = await pending({ ttlMs: -1000 });

    const listed = await ToolApprovalStore.listPending(CHAT_ID);
    assert.ok(!listed.some((r) => r.requestId === row.requestId));
  });

  it('rejects an outcome that is neither approved nor denied', async () => {
    const row = await pending();

    await assert.rejects(() => ToolApprovalStore.resolve(row.requestId, { outcome: 'maybe' }));
  });

  // The double-click / retried-response guard.
  it('lets execution be claimed exactly once', async () => {
    const row = await pending();
    await ToolApprovalStore.resolve(row.requestId, { outcome: 'approved' });

    assert.equal(await ToolApprovalStore.claimExecution(row.requestId), true);
    assert.equal(await ToolApprovalStore.claimExecution(row.requestId), false);
  });

  it('never lets a denied request be claimed for execution', async () => {
    const row = await pending();
    await ToolApprovalStore.resolve(row.requestId, { outcome: 'denied' });

    assert.equal(await ToolApprovalStore.claimExecution(row.requestId), false);
  });

  it('never lets a pending request be claimed for execution', async () => {
    const row = await pending();

    assert.equal(await ToolApprovalStore.claimExecution(row.requestId), false);
  });

  it('reports a turn as fully decided only once nothing is pending', async () => {
    const turnId = `turn-${Date.now()}`;
    const a = await pending({ turnId });
    const b = await pending({ turnId });

    await ToolApprovalStore.resolve(a.requestId, { outcome: 'approved' });
    assert.equal((await ToolApprovalStore.listPendingForTurn(CHAT_ID, turnId)).length, 1);

    await ToolApprovalStore.resolve(b.requestId, { outcome: 'denied' });
    assert.equal((await ToolApprovalStore.listPendingForTurn(CHAT_ID, turnId)).length, 0);
    assert.equal((await ToolApprovalStore.listForTurn(CHAT_ID, turnId)).length, 2);
  });

  it('lists pending rows already past their expiry, for the sweeper', async () => {
    const stale = await pending({ ttlMs: -1000 });
    const fresh = await pending({ ttlMs: 60000 });

    const expired = await ToolApprovalStore.listExpiredUnswept();
    const ids = expired.map((r) => r.requestId);

    assert.ok(ids.includes(stale.requestId));
    assert.ok(!ids.includes(fresh.requestId));
  });

  it('stops listing a row as expired once it has been swept', async () => {
    const stale = await pending({ ttlMs: -1000 });
    await ToolApprovalStore.expireStale();

    const ids = (await ToolApprovalStore.listExpiredUnswept()).map((r) => r.requestId);
    assert.ok(!ids.includes(stale.requestId));
    assert.equal((await ToolApprovalStore.getByRequestId(stale.requestId)).status, 'expired');
  });

  it('collapses rows to one entry per parked turn, so a turn resumes once', () => {
    const turns = ToolApprovalStore.distinctTurns([
      { chatId: 'c1', turnId: 't1' },
      { chatId: 'c1', turnId: 't1' },
      { chatId: 'c1', turnId: 't2' },
      { chatId: 'c2', turnId: 't1' },
    ]);

    assert.deepEqual(turns, [
      { chatId: 'c1', turnId: 't1' },
      { chatId: 'c1', turnId: 't2' },
      { chatId: 'c2', turnId: 't1' },
    ]);
  });

  it('cancels everything still pending in a chat', async () => {
    await pending();
    await ToolApprovalStore.cancelPending(CHAT_ID);

    assert.equal((await ToolApprovalStore.listPending(CHAT_ID)).length, 0);
  });
});

describe('toInputRequest', () => {
  it('exposes the frozen input and hash, and hides orchestration state', async () => {
    const row = await pending({ input: { uav: 3, altitude: 40 } });
    const request = toInputRequest(row);

    assert.equal(request.kind, 'tool-approval');
    assert.equal(request.action.kind, 'tool-call');
    assert.deepEqual(request.action.input, { uav: 3, altitude: 40 });
    assert.equal(request.action.inputHash, row.inputHash);
    assert.ok(request.expiresAt);
    assert.equal(request.resumeContext, undefined);

    await sequelize.models.ChatToolApproval.destroy({ where: { requestId: row.requestId } });
  });
});
