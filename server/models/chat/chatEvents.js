import { eventBus, EVENTS } from '../../common/eventBus.js';

/**
 * Emits an assistant-facing error into a chat over the EventBus.
 *
 * Every failure path in the chat module used to inline the exact same
 * CHAT_ASSISTANT_MESSAGE literal (chat.js tool loop, subAgentManager
 * background start, subAgentManager resume). Three copies of one shape is
 * three chances for them to drift, so the shape lives here now.
 *
 * @param {string} chatId - Chat that should show the error
 * @param {string} content - Human-readable error text
 */
export function emitAssistantError(chatId, content) {
  eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, {
    chatId,
    from: 'assistant',
    timestamp: new Date().toISOString(),
    message: {
      role: 'assistant',
      content,
      type: 'text',
      status: 'error',
    },
  });
}

/**
 * Emits a chat item to clients for every sender the UI renders on the assistant
 * side of the conversation. `subagent` is included: those messages arrive in the
 * user turn protocol-wise, but they are produced by the system, not typed by the
 * user, so the client shows them as assistant-side output.
 *
 * @param {object} chatItem - Chat item as returned by ChatHistoryManager.addMessage
 */
const BROADCAST_SENDERS = new Set(['assistant', 'subagent']);

export function emitAssistantMessage(chatItem) {
  if (BROADCAST_SENDERS.has(chatItem?.from)) {
    eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, chatItem);
  }
}

/**
 * Signals whether a chat is mid-turn, so the UI can disable its input instead of
 * letting the user fire a message into a chat that will just queue it behind a
 * tool loop. Emitted when the per-chat lock is taken and again when it is fully
 * released — which, for a turn with tool calls, is after the LAST iteration, not
 * after the first response.
 *
 * @param {string} chatId
 * @param {boolean} busy
 */
export function emitChatBusy(chatId, busy) {
  eventBus.emitSafe(EVENTS.CHAT_BUSY, { chatId, busy });
}
