import { chatLogger } from '../../common/logger.js';

// In-memory registry of subagent runtime state, keyed by the subagent's own chatId.
// This is orchestration state, not chat content: it does not survive a server
// restart, but neither does subagent execution itself (chat.js's createSubAgent
// runs it as a fire-and-forget promise chain with no resume mechanism), so
// persisting this to the DB would not buy real resilience today.
const registry = new Map();

/**
 * Registers a newly created subagent.
 * @param {Object} params
 * @param {string} params.chatId - Subagent's own chat id
 * @param {string} params.parentChatId - Chat that created this subagent
 * @param {string} params.agentType - Agent profile assigned to the subagent
 * @param {string} params.parentToolName - Name of the tool call in the parent chat that triggered this subagent
 * @param {string} params.firstMessage - Initial message sent to the subagent
 * @param {Object} [params.contextParams] - Fixed context data injected into the subagent's tool calls
 * @returns {Object} The registered record
 */
export function registerSubAgent({
  chatId,
  parentChatId,
  agentType,
  parentToolName,
  firstMessage,
  contextParams = {},
}) {
  const now = new Date().toISOString();
  const record = {
    chatId,
    parentChatId,
    agentType,
    parentToolName,
    firstMessage,
    contextParams: Object.freeze({ ...contextParams }),
    status: 'running',
    error: null,
    createdAt: now,
    updatedAt: now,
  };
  registry.set(chatId, record);
  chatLogger.debug(`[subAgentRegistry] registered ${chatId} (parent=${parentChatId}, type=${agentType})`);
  return record;
}

/**
 * @param {string} chatId
 * @returns {Object|undefined}
 */
export function getSubAgent(chatId) {
  let subAgent = registry.get(chatId);
  if (!subAgent) {
    chatLogger.warn(`[subAgentRegistry] getSubAgent: unknown chatId ${chatId}`);
  }
  return subAgent;
}

/**
 * Fixed context params for a subagent chat. Returns {} for unregistered chats
 * (i.e. regular, non-subagent chats), so callers can merge unconditionally.
 * @param {string} chatId
 * @returns {Object}
 */
export function getContextParams(chatId) {
  return registry.get(chatId)?.contextParams ?? {};
}

/**
 * @param {string} chatId
 * @param {'running'|'done'|'error'} status
 * @param {Object} [options]
 * @param {string} [options.error]
 * @returns {Object|null} Updated record, or null if chatId isn't registered
 */
export function updateSubAgentStatus(chatId, status, { error = null } = {}) {
  const record = registry.get(chatId);
  if (!record) {
    chatLogger.warn(`[subAgentRegistry] updateSubAgentStatus: unknown chatId ${chatId}`);
    return null;
  }
  record.status = status;
  record.error = error;
  record.updatedAt = new Date().toISOString();
  return record;
}

/**
 * @param {string} parentChatId
 * @returns {Array<Object>}
 */
export function listSubAgentsForParent(parentChatId) {
  return Array.from(registry.values()).filter((record) => record.parentChatId === parentChatId);
}

/**
 * @param {string} chatId
 */
export function removeSubAgent(chatId) {
  registry.delete(chatId);
}
