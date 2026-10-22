import { eveClient, explainEveError } from './eveClient.js';
import { EveRelay } from './eveRelay.js';
import { ChatHistoryManager } from './chatHistoryManager.js';
import { InputAudit } from './inputAudit.js';
import {
  emitAssistantError,
  emitAssistantMessage,
  emitChatActivity,
  emitChatBusy,
  emitToolApprovalFailed,
  emitToolApprovalRequested,
  emitToolApprovalResolved,
} from './chatEvents.js';
import { EveDefaultAgent } from '../../config/config.js';
import { chatLogger } from '../../common/logger.js';
import { TURN_PHASE } from './eveEvents.js';

/**
 * Where the relay keeps what it must not lose across a restart, in the chat's metadata:
 * `sessionId` (eve's id, the same slot the legacy provider session uses: a chat runs one engine),
 * `eveAgent` (which eve agent), `eveStreamIndex` (events already consumed) and `eveTurn` (the state
 * of the open turn, including the approvals still waiting for an answer).
 */
const store = {
  async load(chatId) {
    const metadata = await ChatHistoryManager.getChatMetadata(chatId);
    return {
      agent: metadata.eveAgent ?? EveDefaultAgent,
      sessionId: metadata.sessionId ?? null,
      cursor: metadata.eveStreamIndex ?? 0,
      turn: metadata.eveTurn ?? null,
    };
  },
  async save(chatId, { cursor, turn }) {
    await ChatHistoryManager.updateChat(chatId, { metadata: { eveStreamIndex: cursor, eveTurn: turn } });
  },
};

export const eveRelay = new EveRelay({
  client: eveClient,
  store,
  items: { add: (chatId, message) => ChatHistoryManager.addMessage(chatId, 'assistant', message) },
  emit: {
    assistantMessage: emitAssistantMessage,
    assistantError: emitAssistantError,
    busy: emitChatBusy,
    activity: emitChatActivity,
    approvalRequested: emitToolApprovalRequested,
    approvalResolved: emitToolApprovalResolved,
  },
  audit: InputAudit,
});

/**
 * What the orchestrator talks to. An object, not imports used inline, so a test can swap the
 * network and the audit and check the one rule that matters: no audit row, no answer to eve.
 */
export const deps = { client: eveClient, store, relay: eveRelay, audit: InputAudit };

/** The decision could not be written to the audit, so it was NOT sent to eve. */
export class AuditUnavailableError extends Error {
  constructor(cause) {
    super('Could not record the decision, so nothing was sent to the agent. Try again.');
    this.name = 'AuditUnavailableError';
    this.cause = cause;
  }
}

/** A failure of a call to eve, in words the operator can act on (see `explainEveError`). */
const explain = (error, agent) => {
  let base;
  try {
    base = deps.client.baseOf?.(agent);
  } catch {
    // an agent that is not configured: the error being explained already says so
  }
  return explainEveError(error, agent, base);
};

/** Requests answered but not yet confirmed by eve, so a double click cannot send the answer twice. */
const answered = new Map();

/**
 * Bridges chats whose `metadata.engine === 'eve'` to an eve agent (`gcs_eevee_assistant`), running
 * alongside `MessageOrchestrator` — the legacy tool-loop orchestrator is untouched, this is a
 * parallel path selected per chat.
 *
 * It does NOT wait for the answer. A turn can wait on a human for hours, so `processMessage` only
 * sends the message and makes sure the `EveRelay` is reading; what comes back reaches the client
 * through the same events the legacy path uses (assistant messages, busy, approvals).
 */
export class EveOrchestrator {
  /**
   * @param {string} chatId
   * @param {string} message
   * @returns {Promise<{chatId: string, accepted: true}>}
   */
  static async processMessage(chatId, message) {
    emitChatBusy(chatId, true);

    let agent = null;
    try {
      const userItem = await ChatHistoryManager.addMessage(chatId, 'user', {
        role: 'user',
        type: 'text',
        content: message,
      });
      emitAssistantMessage(userItem); // no-op for 'user' (BROADCAST_SENDERS filters it), kept for symmetry

      const stored = await deps.store.load(chatId);
      agent = stored.agent;
      const { sessionId } = stored;
      if (!sessionId) {
        const created = await deps.client.createSession(agent, message);
        await ChatHistoryManager.setSessionId(chatId, created.sessionId);
        await ChatHistoryManager.updateChat(chatId, {
          metadata: { eveStreamIndex: 0, eveTurn: null, eveAgent: agent },
        });
        chatLogger.info(`EveOrchestrator: created eve session ${created.sessionId} (${agent}) for chat ${chatId}`);
      } else {
        await deps.client.sendMessage(agent, sessionId, message);
      }

      this._startReader(chatId);
      return { chatId, accepted: true };
    } catch (error) {
      chatLogger.error(`EveOrchestrator: error sending a message for chat ${chatId}:`, error);
      emitAssistantError(chatId, explain(error, agent));
      emitChatBusy(chatId, false);
      throw error;
    }
  }

  /**
   * Answers the approvals a chat is parked on. ONLY a structured answer decides: free text sent to
   * a session with an approval open CANCELS it (eve outcome `ignored`), it never approves.
   *
   * The decision is written to the audit BEFORE it is sent, and if that write fails nothing is
   * sent: no record, no flight. The approval stays open, so the operator can try again.
   *
   * eve confirms with `input.resolved`, which the relay turns into the `resolved` event the client
   * listens to; this method does not emit it, so the card goes away when eve says it took the answer.
   *
   * @param {string} chatId
   * @param {Array<{requestId: string, optionId: string, text?: string}>} responses
   * @returns {Promise<{sent: string[], stale: Array<{requestId: string, reason: string}>}>}
   */
  static async respondToInput(chatId, responses) {
    const { agent, sessionId, turn } = await deps.store.load(chatId);
    const open = new Map((turn?.pending ?? []).map((request) => [request.requestId, request]));
    const inFlight = answered.get(chatId) ?? new Set();

    const valid = [];
    const stale = [];
    for (const { requestId, optionId, text } of responses ?? []) {
      const request = open.get(requestId);
      if (!request) stale.push({ requestId, reason: 'stale' });
      else if (inFlight.has(requestId)) stale.push({ requestId, reason: 'already_answered' });
      else if (!request.options.some((option) => option.id === optionId))
        stale.push({ requestId, reason: 'invalid_option' });
      else valid.push({ request, response: { requestId, optionId, ...(text ? { text } : {}) } });
    }
    if (valid.length === 0) return { sent: [], stale };

    try {
      for (const { request, response } of valid) {
        if (request.kind !== 'tool-approval') continue;
        // The first option is the affirmative one for every request eve sends (approve / continue).
        await deps.audit.record({
          engine: 'eve',
          chatId,
          requestId: request.requestId,
          callId: request.action?.callId ?? null,
          toolName: request.toolName,
          kind: 'decided',
          source: 'operator',
          outcome: response.optionId === request.options[0]?.id ? 'approved' : 'denied',
          text: response.text ?? null,
        });
      }
    } catch (error) {
      chatLogger.error(`EveOrchestrator: audit write failed for chat ${chatId}, decision NOT sent:`, error);
      const unavailable = new AuditUnavailableError(error);
      emitToolApprovalFailed(
        chatId,
        valid.map(({ response }) => ({ requestId: response.requestId, message: unavailable.message }))
      );
      throw unavailable;
    }

    const ids = valid.map(({ response }) => response.requestId);
    ids.forEach((id) => inFlight.add(id));
    answered.set(chatId, inFlight);

    try {
      await deps.client.sendInputResponses(
        agent,
        sessionId,
        valid.map(({ response }) => response)
      );
    } catch (error) {
      ids.forEach((id) => inFlight.delete(id));
      // The audit says the operator decided, but eve never heard it: leave a row that says so.
      for (const { request } of valid) {
        if (request.kind !== 'tool-approval') continue;
        await deps.audit.recordSafe({
          engine: 'eve',
          chatId,
          requestId: request.requestId,
          callId: request.action?.callId ?? null,
          toolName: request.toolName,
          kind: 'decided',
          source: 'system',
          outcome: 'invalid',
          text: `the decision was not delivered to eve: ${error.message}`,
        });
      }
      emitToolApprovalFailed(
        chatId,
        ids.map((requestId) => ({
          requestId,
          message: `The decision could not be delivered: ${explain(error, agent)}`,
        }))
      );
      throw error;
    }

    this._startReader(chatId);
    return { sent: ids, stale };
  }

  /** Asks eve to stop the open turn. Asynchronous: the end shows up in the stream as `turn.cancelled`. */
  static async cancel(chatId) {
    const { agent, sessionId } = await deps.store.load(chatId);
    if (!sessionId) return;
    await deps.client.cancelSession(agent, sessionId);
    this._startReader(chatId);
  }

  /** Approvals the chat is waiting on, as the client renders them (it survives a reload). */
  static async getPendingApprovals(chatId) {
    const { turn } = await deps.store.load(chatId);
    return (turn?.pending ?? []).map((request) => ({ ...request, chatId }));
  }

  /**
   * Boot: a turn that was running when the process died is still running in eve. Pick its reader
   * back up from the stored cursor. A turn parked on an approval needs nothing: it resumes when
   * the answer is sent.
   */
  static async resumeOpenTurns() {
    const chats = await ChatHistoryManager.getAllChats({ status: 'active', limit: 1000 });
    const open = chats.filter(
      (chat) => chat.metadata?.engine === 'eve' && chat.metadata?.eveTurn?.phase === TURN_PHASE.RUNNING
    );
    for (const chat of open) {
      chatLogger.info(`EveOrchestrator: resuming the open turn of chat ${chat.id}`);
      this._startReader(chat.id);
    }
    return open.length;
  }

  static _startReader(chatId) {
    deps.relay
      .ensure(chatId)
      .catch((error) => chatLogger.error(`EveOrchestrator: reader of chat ${chatId} failed:`, error))
      .finally(() => {
        if (!deps.relay.isRunning(chatId)) answered.delete(chatId);
      });
  }
}
