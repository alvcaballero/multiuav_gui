import { describe, it, before, after } from 'node:test';
import assert from 'node:assert/strict';
import sequelize from '../common/sequelize.js';
import { InputAudit } from '../models/chat/inputAudit.js';
import { MessageOrchestrator } from '../models/chat/chat.js';
import { ToolApprovalStore } from '../models/chat/toolApprovalStore.js';
import { freezeToolCall } from '../models/chat/toolApproval.js';

// Real SQL on purpose: the unique index is what makes replaying an eve stream safe, and an
// in-memory fake would prove nothing about it.
const CHAT_ID = `test-audit-${process.pid}-${Date.now()}`;

const row = (overrides = {}) => ({
  engine: 'eve',
  chatId: CHAT_ID,
  requestId: `req-${Math.random()}`,
  callId: 'call-1',
  toolName: 'start_mission',
  kind: 'requested',
  source: 'eve',
  ...overrides,
});

describe('InputAudit', () => {
  before(async () => {
    await sequelize.models.ChatInputAudit.sync();
    await sequelize.models.ChatToolApproval.sync();
  });

  after(async () => {
    await sequelize.models.ChatInputAudit.destroy({ where: { chatId: CHAT_ID } });
    await sequelize.models.ChatToolApproval.destroy({ where: { chatId: CHAT_ID } });
  });

  it('records an event once and treats a replay of the same event as a no-op', async () => {
    const event = row({ kind: 'decided', source: 'eve', outcome: 'denied', input: { deviceId: 2 } });

    assert.equal(await InputAudit.record(event), true);
    assert.equal(await InputAudit.record(event), false, 'eve replays events after a restart: no second row');

    const stored = (await InputAudit.listForChat(CHAT_ID)).filter((r) => r.requestId === event.requestId);
    assert.equal(stored.length, 1);
    assert.equal(stored[0].responderPrincipalId, null, 'the GCS has no users');
  });

  it('keeps the operator and eve answers to the same request as two rows, so a mismatch shows', async () => {
    const base = row({ kind: 'decided' });
    await InputAudit.record({ ...base, source: 'operator', outcome: 'approved' });
    await InputAudit.record({ ...base, source: 'eve', outcome: 'ignored' });

    const stored = (await InputAudit.listForChat(CHAT_ID)).filter((r) => r.requestId === base.requestId);
    assert.deepEqual(
      stored.map((r) => [r.source, r.outcome]),
      [
        ['operator', 'approved'],
        ['eve', 'ignored'],
      ]
    );
  });

  it('`record` throws on a bad row and `recordSafe` swallows it, because only one of them may gate an action', async () => {
    const bad = row({ kind: 'not-a-kind' });

    await assert.rejects(() => InputAudit.record(bad));
    assert.equal(await InputAudit.recordSafe(bad), false);
  });
});

describe('InputAudit — the legacy engine writes to it too (state stays in ChatToolApproval)', () => {
  const gated = {
    type: 'function_call',
    name: 'start_mission',
    call_id: 'call-legacy',
    arguments: '{"uav":3,"altitude":40}',
  };
  let resumed;
  const original = MessageOrchestrator._resumeParkedTurn;

  before(async () => {
    await sequelize.models.ChatInputAudit.sync();
    await sequelize.models.ChatToolApproval.sync();
    // Resuming a parked turn calls the LLM; the audit does not depend on it.
    MessageOrchestrator._resumeParkedTurn = async (chatId, turnId) => {
      resumed = { chatId, turnId };
    };
  });

  after(async () => {
    MessageOrchestrator._resumeParkedTurn = original;
    await sequelize.models.ChatInputAudit.destroy({ where: { chatId: CHAT_ID } });
    await sequelize.models.ChatToolApproval.destroy({ where: { chatId: CHAT_ID } });
  });

  it('records what was asked (with the input and its hash), what was decided and that it ran', async () => {
    const chatId = `${CHAT_ID}-flow`;
    const parked = await MessageOrchestrator._parkForApproval(
      chatId,
      [gated],
      { allowedTools: null, agent: { name: 'x' } },
      { turnId: 't1', iteration: 1 }
    );
    assert.equal(parked, true);

    const [approval] = await ToolApprovalStore.listPending(chatId);
    const { resolutions } = await MessageOrchestrator.respondToApproval(
      chatId,
      [{ requestId: approval.requestId, optionId: 'approve' }],
      { responderPrincipalId: null }
    );
    assert.equal(resolutions[0].outcome, 'approved');
    assert.deepEqual(resumed, { chatId, turnId: 't1' });

    const decided = await ToolApprovalStore.getByRequestId(approval.requestId);
    assert.equal(
      await MessageOrchestrator._denyUnapprovedToolCall({
        callId: decided.callId,
        name: 'start_mission',
        input: decided.input,
        approval: decided,
      }),
      null
    );

    const stored = await InputAudit.listForChat(chatId);
    assert.deepEqual(
      stored.map((r) => [r.engine, r.kind, r.source, r.outcome]),
      [
        ['legacy', 'requested', 'system', null],
        ['legacy', 'decided', 'operator', 'approved'],
        ['legacy', 'executed', 'system', null],
      ]
    );
    assert.deepEqual(stored[0].input, { uav: 3, altitude: 40 });
    assert.ok(stored[0].inputHash);

    await sequelize.models.ChatInputAudit.destroy({ where: { chatId } });
    await sequelize.models.ChatToolApproval.destroy({ where: { chatId } });
  });

  it('records the denial with its reason, and an approval that expired unanswered', async () => {
    const chatId = `${CHAT_ID}-deny`;
    await MessageOrchestrator._parkForApproval(
      chatId,
      [gated],
      { allowedTools: null, agent: null },
      { turnId: 't2', iteration: 1 }
    );
    const [approval] = await ToolApprovalStore.listPending(chatId);
    await MessageOrchestrator.respondToApproval(chatId, [
      { requestId: approval.requestId, optionId: 'deny', text: 'viento' },
    ]);

    // A second one nobody answers, already past its expiry.
    await ToolApprovalStore.createPending({
      chatId,
      turnId: 't3',
      iteration: 1,
      frozen: freezeToolCall({ callId: 'call-late', toolName: 'start_mission', input: { uav: 1 }, ttlMs: -1000 }),
      resumeContext: { allowedTools: null, toolCalls: [] },
    });
    await MessageOrchestrator.sweepExpiredApprovals();

    const stored = await InputAudit.listForChat(chatId);
    const denied = stored.find((r) => r.outcome === 'denied');
    assert.equal(denied.text, 'viento');
    assert.equal(denied.source, 'operator');
    const expired = stored.find((r) => r.outcome === 'expired');
    assert.equal(expired.source, 'system');
    assert.equal(expired.kind, 'decided');

    await sequelize.models.ChatInputAudit.destroy({ where: { chatId } });
    await sequelize.models.ChatToolApproval.destroy({ where: { chatId } });
  });
});
