import sequelize from '../../common/sequelize.js';
import { Op } from 'sequelize';
import { chatLogger } from '../../common/logger.js';

const model = () => sequelize.models.ChatToolApproval;

const toPlain = (row) => (row ? row.get({ plain: true }) : null);

/**
 * The shape the client is given. Deliberately NOT the raw row: `resumeContext`
 * is orchestration state, not something an operator decides on.
 */
export const toInputRequest = (row) => ({
  requestId: row.requestId,
  kind: 'tool-approval',
  chatId: row.chatId,
  toolName: row.toolName,
  action: {
    kind: 'tool-call',
    callId: row.callId,
    toolName: row.toolName,
    input: row.input,
    inputHash: row.inputHash,
  },
  expiresAt: new Date(row.expiresAt).toISOString(),
  status: row.status,
});

export class ToolApprovalStore {
  static async createPending({ chatId, turnId, iteration, frozen, resumeContext = null }) {
    await model().create({
      requestId: frozen.requestId,
      chatId,
      callId: frozen.callId,
      toolName: frozen.toolName,
      input: frozen.input,
      inputHash: frozen.inputHash,
      status: 'pending',
      turnId,
      iteration,
      resumeContext,
      frozenAt: frozen.frozenAt,
      expiresAt: frozen.expiresAt,
    });

    chatLogger.info(`[ToolApproval] Pending ${frozen.toolName} (request ${frozen.requestId}) on chat ${chatId}`);

    // Read back rather than return the created instance: `create` omits the
    // columns it was not given, so its shape differs from every other read here.
    return this.getByRequestId(frozen.requestId);
  }

  static async getByRequestId(requestId) {
    return toPlain(await model().findByPk(requestId));
  }

  /**
   * Still-pending, still-valid requests. Used to re-project the approval state
   * after a reload: the client asks, the server answers from the DB, so a
   * pending approval survives a refresh without any client-side storage.
   */
  static async listPending(chatId, now = new Date()) {
    const rows = await model().findAll({
      where: { chatId, status: 'pending', expiresAt: { [Op.gt]: now } },
      order: [['frozenAt', 'ASC']],
    });
    return rows.map(toPlain);
  }

  static async listPendingForTurn(chatId, turnId) {
    const rows = await model().findAll({ where: { chatId, turnId, status: 'pending' } });
    return rows.map(toPlain);
  }

  static async listForTurn(chatId, turnId) {
    const rows = await model().findAll({ where: { chatId, turnId }, order: [['frozenAt', 'ASC']] });
    return rows.map(toPlain);
  }

  /**
   * Resolves a request, atomically. Returns null when the row was already
   * resolved, cancelled or expired — a second answer NEVER re-opens a decision,
   * which is also what makes concurrent operators safe: the first write wins and
   * the losers get null.
   */
  static async resolve(requestId, { outcome, responderPrincipalId = null, denyReason = null, now = new Date() }) {
    if (outcome !== 'approved' && outcome !== 'denied') {
      throw new Error(`ToolApprovalStore.resolve: invalid outcome "${outcome}"`);
    }

    const [updated] = await model().update(
      { status: outcome, responderPrincipalId, denyReason, resolvedAt: now },
      { where: { requestId, status: 'pending', expiresAt: { [Op.gt]: now } } }
    );

    if (updated === 0) {
      chatLogger.warn(`[ToolApproval] Stale response for request ${requestId} — already resolved or expired`);
      return null;
    }

    return this.getByRequestId(requestId);
  }

  /**
   * Claims the right to execute, atomically. Returns false if this call already
   * ran — the guard against a double click, a retried response, or a resume that
   * overlaps an in-flight execution.
   */
  static async claimExecution(requestId, now = new Date()) {
    const [updated] = await model().update(
      { executedAt: now },
      { where: { requestId, status: 'approved', executedAt: null } }
    );
    return updated === 1;
  }

  /** Still marked pending, but past their expiry — the sweeper's input. */
  static async listExpiredUnswept(now = new Date()) {
    const rows = await model().findAll({ where: { status: 'pending', expiresAt: { [Op.lte]: now } } });
    return rows.map(toPlain);
  }

  static distinctTurns(rows) {
    const seen = new Map();
    for (const { chatId, turnId } of rows) seen.set(`${chatId}::${turnId}`, { chatId, turnId });
    return [...seen.values()];
  }

  static async expireStale(now = new Date()) {
    const [updated] = await model().update(
      { status: 'expired', resolvedAt: now },
      { where: { status: 'pending', expiresAt: { [Op.lte]: now } } }
    );
    if (updated > 0) chatLogger.info(`[ToolApproval] Expired ${updated} stale request(s)`);
    return updated;
  }

  static async cancelPending(chatId, now = new Date()) {
    const [updated] = await model().update(
      { status: 'cancelled', resolvedAt: now },
      { where: { chatId, status: 'pending' } }
    );
    return updated;
  }
}
