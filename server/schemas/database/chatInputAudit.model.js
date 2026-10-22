import { Model, DataTypes, Sequelize } from 'sequelize';

const ChatInputAudit_TABLE = 'ChatInputAudit';

export const AUDIT_ENGINES = ['legacy', 'eve'];
export const AUDIT_KINDS = ['requested', 'decided', 'executed'];
export const AUDIT_SOURCES = ['operator', 'system', 'eve'];
export const AUDIT_OUTCOMES = ['approved', 'denied', 'expired', 'cancelled', 'ignored', 'invalid'];

/**
 * Append-only record of what a human was asked to decide about a tool call, what was decided,
 * and whether it ran. One row per event, never updated.
 *
 * Not the approval state: `ChatToolApproval` is that (it is overwritten and carries what a parked
 * turn needs to resume). This table is what is left to read after an incident, so it has no FK to
 * `Chat` — deleting a chat must not delete the record of what it armed.
 *
 * `responderPrincipalId` stays null: the GCS has no users.
 */
const ChatInputAuditSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  engine: {
    allowNull: false,
    type: DataTypes.ENUM(...AUDIT_ENGINES),
    validate: { isIn: [AUDIT_ENGINES] },
  },
  chatId: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  requestId: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  callId: {
    allowNull: true,
    type: DataTypes.STRING,
  },
  toolName: {
    allowNull: true,
    type: DataTypes.STRING,
  },
  kind: {
    allowNull: false,
    type: DataTypes.ENUM(...AUDIT_KINDS),
    validate: { isIn: [AUDIT_KINDS] },
  },
  source: {
    allowNull: false,
    type: DataTypes.ENUM(...AUDIT_SOURCES),
    validate: { isIn: [AUDIT_SOURCES] },
    comment: 'Who produced the row: the operator answering, the GCS itself (expiry, cancel) or eve confirming.',
  },
  outcome: {
    allowNull: true,
    type: DataTypes.ENUM(...AUDIT_OUTCOMES),
    validate: { isIn: [AUDIT_OUTCOMES] },
    comment: 'Set on `decided` rows. `expired` only exists in the legacy engine, `ignored` only in eve.',
  },
  input: {
    allowNull: true,
    type: DataTypes.JSON,
    comment: 'The exact input the operator was shown. Written on `requested` rows.',
  },
  inputHash: {
    allowNull: true,
    type: DataTypes.STRING,
    comment: 'Legacy only: eve has no input hash.',
  },
  eveEventId: {
    allowNull: true,
    type: DataTypes.STRING,
    comment: 'meta.id of the eve event this row came from.',
  },
  responderPrincipalId: {
    allowNull: true,
    type: DataTypes.STRING,
  },
  text: {
    allowNull: true,
    type: DataTypes.TEXT,
    comment: 'The operator reason for a denial, or the result of an execution.',
  },
  at: {
    allowNull: false,
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
};

class ChatInputAudit extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: ChatInputAudit_TABLE,
      modelName: 'ChatInputAudit',
      timestamps: false,
      indexes: [
        // Re-reading an eve stream after a restart replays events: the same row must not land twice.
        { unique: true, fields: ['engine', 'requestId', 'kind', 'source'] },
        { fields: ['chatId'] },
        { fields: ['callId'] },
      ],
    };
  }
}

export { ChatInputAudit_TABLE, ChatInputAuditSchema, ChatInputAudit };
