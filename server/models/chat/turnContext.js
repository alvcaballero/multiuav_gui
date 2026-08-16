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
    const persistence = ChatHistoryManager.getSessionPersistence();

    const agent = await resolveAgentForChat(chatId);
    const allowedTools = optionsAllowedTools ?? agent.allowedTools;
    chatLogger.debug(`Using agent '${agent.name}' with tools: ${allowedTools ? allowedTools.join(', ') : 'all'}`);

    const tools = getTools(allowedTools);
    const sessionId = await this._resolveSession(chatId, persistence, llmHandler);
    const systemInstructions = await this._resolveSystemPrompt(chatId, agent);

    return new TurnContext({
      chatId,
      agent,
      allowedTools,
      tools,
      sessionId,
      systemInstructions,
      persistence,
    });
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
   * - Empty history: this is a new chat, so the built prompt is persisted as
   *   the first message.
   * - Existing history: the stored prompt wins, so a server restart or an
   *   agent definition edit can't silently change an ongoing conversation.
   *
   * A DB failure degrades to "treat as new chat" rather than killing the turn.
   */
  static async _resolveSystemPrompt(chatId, agent) {
    let systemInstructions = buildSystemPrompt({
      systemPrompt: agent.systemPrompt,
      contextLines: [`session_chat_id: ${chatId}`],
    });

    let history = [];
    try {
      history = await ChatHistoryManager.loadHistory(chatId);
    } catch (error) {
      chatLogger.error('Error loading history from DB:', error);
    }

    if (history.length === 0 && systemInstructions) {
      await ChatHistoryManager.addMessage(chatId, 'system', { role: 'system', content: systemInstructions });
      return systemInstructions;
    }

    const systemMsg = history.find((item) => (item.message || item).role === 'system');
    if (systemMsg) {
      systemInstructions = (systemMsg.message || systemMsg).content;
    }
    return systemInstructions;
  }
}
