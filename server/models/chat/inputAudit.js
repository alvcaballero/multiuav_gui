import sequelize from '../../common/sequelize.js';
import { chatLogger } from '../../common/logger.js';

const model = () => sequelize.models.ChatInputAudit;

/**
 * Append-only audit of the decisions taken on tool calls that move an aircraft, for both engines.
 * `ChatToolApproval` stays the legacy approval STATE; this is the record that outlives it.
 *
 * Two ways to write, because the two have different consequences when the write fails:
 * - `record` throws. Used where a missing row must stop the action (eve `decided`: no row, no POST).
 * - `recordSafe` logs and carries on. Used everywhere else: `requested` is only information, and
 *   `executed` is written after the tool already ran, so there is nothing left to stop.
 */
export class InputAudit {
  /**
   * Inserts one event. The unique index (engine, requestId, kind, source) makes a replay a no-op.
   * @param {object} row - see schemas/database/chatInputAudit.model.js
   * @returns {Promise<boolean>} true if inserted, false if that exact event was already recorded
   */
  static async record(row) {
    try {
      await model().create({ ...row, at: row.at ?? new Date() });
      return true;
    } catch (error) {
      if (error?.name === 'SequelizeUniqueConstraintError') return false;
      throw error;
    }
  }

  /** Same as `record`, but a failure is logged instead of thrown. Never use it to gate an action. */
  static async recordSafe(row) {
    try {
      return await this.record(row);
    } catch (error) {
      chatLogger.error(
        `[InputAudit] Could not record ${row.kind}/${row.source} for ${row.requestId}: ${error.message}`
      );
      return false;
    }
  }

  static async listForChat(chatId) {
    const rows = await model().findAll({ where: { chatId }, order: [['id', 'ASC']] });
    return rows.map((row) => row.get({ plain: true }));
  }
}
