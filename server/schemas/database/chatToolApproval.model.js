import { Model, DataTypes, Sequelize } from 'sequelize';

const ChatToolApproval_TABLE = 'ChatToolApproval';

export const APPROVAL_STATUSES = ['pending', 'approved', 'denied', 'expired', 'cancelled'];

/**
 * A tool call that moves an aircraft, parked until a human decides.
 *
 * `input` + `inputHash` are the POINT of this table: the operator approves one
 * exact serialised input, and that is what runs. The turn is never resumed by
 * re-asking the model what it wanted — a model asked twice can answer twice.
 *
 * `resumeContext` carries what the parked turn needs to continue without
 * re-deriving it (agent, allowed tools, the whole tool-call batch), because the
 * process may restart between the request and the answer.
 */
const ChatToolApprovalSchema = {
  requestId: {
    allowNull: false,
    primaryKey: true,
    type: DataTypes.STRING,
    comment: 'Identity of the approval request. The operator answer is keyed by this, never by free text.',
  },
  chatId: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  callId: {
    allowNull: false,
    type: DataTypes.STRING,
    comment: 'The tool call this authorises. Also the idempotency key for execution.',
  },
  toolName: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  input: {
    allowNull: false,
    type: DataTypes.JSON,
    comment: 'FROZEN resolved input (context params already merged). This exact value is what executes.',
  },
  inputHash: {
    allowNull: false,
    type: DataTypes.STRING,
    comment: 'sha256 of the canonicalised input. Re-checked immediately before execution.',
  },
  status: {
    allowNull: false,
    type: DataTypes.ENUM(...APPROVAL_STATUSES),
    defaultValue: 'pending',
  },
  turnId: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  iteration: {
    allowNull: false,
    type: DataTypes.INTEGER,
    defaultValue: 0,
  },
  resumeContext: {
    allowNull: true,
    type: DataTypes.JSON,
    comment: 'What the parked turn needs to resume: agent, allowedTools, the pending tool-call batch.',
  },
  responderPrincipalId: {
    allowNull: true,
    type: DataTypes.STRING,
    comment: 'WHO resolved it. Null while pending. Required for post-incident audit.',
  },
  denyReason: {
    allowNull: true,
    type: DataTypes.TEXT,
  },
  frozenAt: {
    allowNull: false,
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
  expiresAt: {
    allowNull: false,
    type: DataTypes.DATE,
    comment: 'An approval authorises an action under conditions; past this, the conditions are no longer assumed.',
  },
  resolvedAt: {
    allowNull: true,
    type: DataTypes.DATE,
  },
  executedAt: {
    allowNull: true,
    type: DataTypes.DATE,
    comment: 'Set once the tool actually ran. Guards against double execution on retry or reconnect.',
  },
};

class ChatToolApproval extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: ChatToolApproval_TABLE,
      modelName: 'ChatToolApproval',
      timestamps: false,
      indexes: [{ fields: ['chatId'] }, { fields: ['chatId', 'status'] }, { fields: ['callId'] }],
    };
  }
}

export { ChatToolApproval_TABLE, ChatToolApprovalSchema, ChatToolApproval };
