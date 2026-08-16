import { Model, DataTypes, Sequelize } from 'sequelize';

const ChatUsage_TABLE = 'ChatUsage';

/**
 * One row per LLM request (NOT per ChatMessage).
 *
 * A single user message fans out into many requests: the initial call plus one
 * per recursion of MessageOrchestrator._runTurn while the model keeps asking for
 * tools. They all share a
 * `turnId`, so `GROUP BY turnId` gives the real cost of answering the user,
 * `GROUP BY chatId` the cost of the whole conversation, and a single row the
 * cost of one request.
 *
 * Image tokens are NOT tracked separately: every provider folds them into its
 * prompt/input count, so `inputTokens` already includes them.
 */
const ChatUsageSchema = {
  id: {
    allowNull: false,
    autoIncrement: true,
    primaryKey: true,
    type: DataTypes.INTEGER,
  },
  chatId: {
    allowNull: false,
    type: DataTypes.STRING,
  },
  turnId: {
    allowNull: false,
    type: DataTypes.STRING,
    comment: 'Groups every request triggered by one user message (initial call + tool loop iterations)',
  },
  responseId: {
    type: DataTypes.STRING,
    allowNull: true,
    comment: 'Provider response id when available (null for Gemini/Ollama)',
  },
  provider: {
    type: DataTypes.STRING,
    allowNull: true,
  },
  model: {
    type: DataTypes.STRING,
    allowNull: true,
  },
  agent: {
    type: DataTypes.STRING,
    allowNull: true,
    comment: 'Agent profile name that issued the request (planner, default, ...)',
  },
  phase: {
    type: DataTypes.STRING,
    allowNull: true,
    comment: "'initial' for the first call of a turn, 'tool_loop' for continuations",
  },
  iteration: {
    type: DataTypes.INTEGER,
    allowNull: true,
    defaultValue: 0,
    comment: '0 for the initial call, 1..N for each tool loop iteration',
  },
  inputTokens: {
    type: DataTypes.INTEGER,
    allowNull: true,
  },
  cachedTokens: {
    type: DataTypes.INTEGER,
    allowNull: true,
    comment: 'Cache-read prompt tokens. Subset of inputTokens on OpenAI/Gemini, additive on Anthropic',
  },
  outputTokens: {
    type: DataTypes.INTEGER,
    allowNull: true,
  },
  reasoningTokens: {
    type: DataTypes.INTEGER,
    allowNull: true,
    comment: 'Thinking/reasoning tokens, billed as output',
  },
  totalTokens: {
    type: DataTypes.INTEGER,
    allowNull: true,
  },
  raw: {
    type: DataTypes.JSON,
    allowNull: true,
    comment: 'Provider usage payload verbatim — source of truth if normalization is ever wrong',
  },
  timestamp: {
    allowNull: false,
    type: DataTypes.DATE,
    defaultValue: Sequelize.NOW,
  },
};

class ChatUsage extends Model {
  static associate() {
    // associate
  }

  static config(sequelize) {
    return {
      sequelize,
      tableName: ChatUsage_TABLE,
      modelName: 'ChatUsage',
      timestamps: false,
      indexes: [{ fields: ['chatId'] }, { fields: ['chatId', 'turnId'] }, { fields: ['timestamp'] }],
    };
  }
}

export { ChatUsage_TABLE, ChatUsageSchema, ChatUsage };
