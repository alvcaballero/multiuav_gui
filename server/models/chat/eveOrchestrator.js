import { createEveSession, sendEveMessage, streamEveEvents } from './eveClient.js';
import { ChatHistoryManager } from './chatHistoryManager.js';
import { emitAssistantError, emitAssistantMessage, emitChatBusy } from './chatEvents.js';
import { chatLogger } from '../../common/logger.js';

/**
 * Bridges chats whose `metadata.engine === 'eve'` to the eve agent
 * (`gcs_eevee_assistant`), running fully alongside `MessageOrchestrator` — the
 * legacy tool-loop orchestrator is untouched, this is a parallel path selected
 * per chat. See `client/CLAUDE.md`/`server/CLAUDE.md` for the integration status.
 *
 * MVP scope: projects only the final assistant text of a turn into
 * `CHAT_ASSISTANT_MESSAGE`, same as the legacy path's plain-text replies.
 * Tool-call visibility, HITL approval and subagent progress events are NOT
 * projected yet — the stream carries them (`actions.requested`,
 * `input.requested`, `subagent.called`, ...) but nothing reads them here.
 *
 * The eve session id reuses the same `metadata.sessionId` slot
 * `ChatHistoryManager.getSessionId`/`setSessionId` use for legacy provider
 * sessions — safe because a chat only ever runs one engine, never both.
 *
 * eve's stream is durable and replays from event 0 on every GET unless told
 * otherwise (see `eveClient.streamEveEvents`), so this also tracks a
 * per-chat read cursor in `metadata.eveStreamIndex` — the absolute count of
 * events already consumed — and resumes from it on every turn after the
 * first. Without this, every follow-up message would re-read turn 1's
 * events and `session.waiting` boundary before ever seeing its own.
 */
export class EveOrchestrator {
  /**
   * @param {string} chatId
   * @param {string} message
   * @returns {Promise<{chatId: string, message: string}>}
   */
  static async processMessage(chatId, message) {
    emitChatBusy(chatId, true);

    try {
      const userItem = await ChatHistoryManager.addMessage(chatId, 'user', {
        role: 'user',
        type: 'text',
        content: message,
      });
      emitAssistantMessage(userItem); // no-op for 'user' (BROADCAST_SENDERS filters it), kept for symmetry

      let sessionId = await ChatHistoryManager.getSessionId(chatId);
      let startIndex = 0;
      if (!sessionId) {
        const created = await createEveSession(message);
        sessionId = created.sessionId;
        await ChatHistoryManager.setSessionId(chatId, sessionId);
        chatLogger.info(`EveOrchestrator: created eve session ${sessionId} for chat ${chatId}`);
      } else {
        const metadata = await ChatHistoryManager.getChatMetadata(chatId);
        startIndex = metadata.eveStreamIndex ?? 0;
        await sendEveMessage(sessionId, message);
      }

      const { finalText, nextIndex } = await this._drainToFinalText(sessionId, startIndex);
      await ChatHistoryManager.updateChat(chatId, { metadata: { eveStreamIndex: nextIndex } });

      const chatItem = await ChatHistoryManager.addMessage(chatId, 'assistant', {
        role: 'assistant',
        type: 'text',
        content: finalText,
      });
      emitAssistantMessage(chatItem);

      return { chatId, message: finalText };
    } catch (error) {
      chatLogger.error(`EveOrchestrator: error processing message for chat ${chatId}:`, error);
      emitAssistantError(chatId, error.message || 'Error talking to the eve agent');
      throw error;
    } finally {
      emitChatBusy(chatId, false);
    }
  }

  /**
   * Reads an eve session's NDJSON stream, from `startIndex`, until the turn
   * settles, keeping the text of the last `message.completed` event — turns
   * append one such event per step (tool-call reasoning, then the real
   * answer), so the last one wins.
   * @param {string} sessionId
   * @param {number} startIndex
   * @returns {Promise<{finalText: string, nextIndex: number}>}
   */
  static async _drainToFinalText(sessionId, startIndex) {
    let finalText = '';
    let consumed = 0;
    const TERMINAL = new Set(['session.waiting', 'session.completed', 'session.failed']);

    for await (const event of streamEveEvents(sessionId, { startIndex })) {
      consumed++;
      if (event.type === 'message.completed' && typeof event.data?.message === 'string') {
        finalText = event.data.message;
      }
      if (event.type === 'session.failed' || event.type === 'turn.failed') {
        chatLogger.warn(`EveOrchestrator: session ${sessionId} reported ${event.type}`, event.data);
      }
      if (TERMINAL.has(event.type)) break;
    }

    return { finalText, nextIndex: startIndex + consumed };
  }
}
