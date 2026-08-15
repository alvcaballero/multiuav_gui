import { chatLogger } from '../../common/logger.js';
import { resolveAgentForChat } from './agents/index.js';
import { ChatHistoryManager } from './chatHistoryManager.js';

/**
 * Builds the system prompt sent to the provider as instructions.
 *
 * Single source of truth for the prompt envelope. Both the main chat and
 * subagents used to build this string inline with slightly different
 * separators and key casing ("Session context:" vs "Session_context:"),
 * which is how two formats of the same concept start drifting apart.
 *
 * The envelope is unified here; the *content* of the context stays
 * caller-specific through `contextLines` and `trailer`.
 *
 * @param {object} params
 * @param {string} params.systemPrompt - Agent body prompt
 * @param {string[]} params.contextLines - Session context lines, without the leading "- "
 * @param {string} [params.extraContext] - Free-form text appended inside the context block,
 *   already formatted by the caller (no "- " prefix is added)
 * @param {string} [params.trailer] - Optional text appended after the context block
 * @returns {string|null} Full instructions, or null when the agent has no prompt
 */
export function buildSystemPrompt({ systemPrompt, contextLines = [], extraContext = '', trailer = '' }) {
  if (!systemPrompt) return null;

  const lines = contextLines.map((line) => `- ${line}`);
  if (extraContext) lines.push(extraContext);

  let out = `${systemPrompt}\n\n---\nSession context:\n${lines.join('\n')}`;
  if (trailer) out += `\n${trailer}`;
  return out;
}

/**
 * Everything one turn needs to know, resolved once.
 *
 * This is a DTO, not a service: it holds data and has no business methods.
 * Resolution order matters and is encoded in `build()` — the system prompt
 * step both reads and seeds the history snapshot, so it must run after the
 * history is loaded and before the user message is persisted.
 */
export class TurnContext {
  constructor(fields) {
    Object.assign(this, fields);
    Object.freeze(this);
  }

  /**
   * Resolves agent, tools, session and system instructions for a chat turn.
   *
   * @param {string} chatId
   * @param {object} options
   * @param {Array<string>|null} [options.allowedTools] - Overrides the agent's tool list
   * @param {Function} options.getTools - (allowedTools) => Array, provider tool resolver
   * @param {object} options.llmHandler - Active LLM handler (for ensureSession)
   * @returns {Promise<TurnContext>}
   */
  static async build(chatId, { allowedTools: optionsAllowedTools = null, getTools, llmHandler } = {}) {
    const historySnapshot = await this._loadHistory(chatId);
    const persistence = ChatHistoryManager.getSessionPersistence();

    const agent = await resolveAgentForChat(chatId);
    const allowedTools = optionsAllowedTools ?? agent.allowedTools;
    chatLogger.debug(`Using agent '${agent.name}' with tools: ${allowedTools ? allowedTools.join(', ') : 'all'}`);

    const tools = getTools(allowedTools);
    const sessionId = await this._resolveSession(chatId, persistence, llmHandler);
    const systemInstructions = await this._resolveSystemPrompt(chatId, agent, historySnapshot);

    return new TurnContext({
      chatId,
      agent,
      allowedTools,
      tools,
      sessionId,
      systemInstructions,
      historySnapshot,
      persistence,
    });
  }

  /**
   * History snapshot taken BEFORE the user message is persisted, so it can
   * serve as full context on the no-session fallback path.
   * A DB failure degrades to an empty history rather than killing the turn.
   */
  static async _loadHistory(chatId) {
    try {
      const history = await ChatHistoryManager.loadHistory(chatId);
      chatLogger.info(`📂 Loaded ${history.length} messages from DB for chat: ${chatId}`);
      return history;
    } catch (error) {
      chatLogger.error('Error loading history from DB:', error);
      return [];
    }
  }

  /**
   * Resolves the provider session, or null to force the full-history path.
   *
   * Forked chats carry copied history the provider never saw. On the first
   * turn of a fork the session is bypassed so the whole snapshot is sent;
   * after that turn the provider holds a session seeded with full context.
   */
  static async _resolveSession(chatId, persistence, llmHandler) {
    const chatMeta = await ChatHistoryManager.getChatMetadata(chatId);
    const isUnseededFork = !!(chatMeta.forkedFrom && !chatMeta.sessionId);

    if (isUnseededFork) {
      chatLogger.info(`[fork] First turn of forked chat ${chatId} — using full-history path to seed provider context`);
      return null;
    }
    return llmHandler.ensureSession(chatId, persistence);
  }

  /**
   * Resolves the system instructions and keeps history and DB in sync.
   *
   * Two paths, mutually exclusive:
   * - Empty history: the prompt is persisted and PUSHED INTO `historySnapshot`
   *   (mutation is intentional — the caller's snapshot must contain it).
   * - Existing history: the stored prompt wins, so a server restart or an
   *   agent definition edit can't silently change an ongoing conversation.
   */
  static async _resolveSystemPrompt(chatId, agent, historySnapshot) {
    let systemInstructions = buildSystemPrompt({
      systemPrompt: agent.systemPrompt,
      contextLines: [`session_chat_id: ${chatId}`],
    });

    if (historySnapshot.length === 0 && systemInstructions) {
      const systemItem = await ChatHistoryManager.addMessage(chatId, 'system', {
        role: 'system',
        content: systemInstructions,
      });
      historySnapshot.push(systemItem);
      return systemInstructions;
    }

    const systemMsg = historySnapshot.find((item) => (item.message || item).role === 'system');
    if (systemMsg) {
      systemInstructions = (systemMsg.message || systemMsg).content;
    }
    return systemInstructions;
  }
}
