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
 * Emits a chat item to clients, but only when it comes from the assistant.
 * Mirrors the guard MessageOrchestrator.emitAssistantMessage applies.
 *
 * @param {object} chatItem - Chat item as returned by ChatHistoryManager.addMessage
 */
export function emitAssistantMessage(chatItem) {
  if (chatItem?.from === 'assistant') {
    eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, chatItem);
  }
}
