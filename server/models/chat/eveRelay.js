import { EveHttpError, EveSessionExpiredError } from './eveClient.js';
import {
  activityOf,
  initialTurnState,
  isReadComplete,
  NOISE_EVENTS,
  reduceEvent,
  toAuditOutcome,
  TURN_PHASE,
} from './eveEvents.js';
import { chatLogger } from '../../common/logger.js';

// The cursor and turn state are saved on every event that changes something, and at least this
// often while only noise (text deltas) goes by, so a restart never re-reads more than this.
const SAVE_EVERY = 100;
const RECONNECT_BACKOFF_MS = [1000, 2000, 5000, 10000, 30000];
const TURN_END = new Set([TURN_PHASE.COMPLETED, TURN_PHASE.FAILED, TURN_PHASE.CANCELLED]);

/** The part of the turn state worth keeping across a restart. */
export const serializeTurn = (state) => ({
  phase: state.phase,
  lastMessage: state.lastMessage,
  pending: state.pending,
  tasks: state.tasks,
  hiddenCalls: state.hiddenCalls,
  approvalCalls: state.approvalCalls,
});

/**
 * The state to start a read from. A finished turn starts over: otherwise the `session.waiting`
 * that closes it would settle the NEXT turn before it began. A turn parked on a human is running
 * again, because the only reason to read it again is that something was sent to it.
 */
const restoreTurn = (turn) => {
  if (!turn || TURN_END.has(turn.phase)) return initialTurnState();
  const restored = { ...initialTurnState(), ...turn, settled: false };
  if (restored.phase === TURN_PHASE.AWAITING_INPUT) restored.phase = TURN_PHASE.RUNNING;
  return restored;
};

const asText = (output) => (typeof output === 'string' ? output : JSON.stringify(output ?? null));

/** Worth reconnecting for: the network or a 5xx. A 4xx or an expired session will not fix itself. */
const isTransient = (error) => {
  if (error instanceof EveSessionExpiredError) return false;
  if (error instanceof EveHttpError)
    return error.status >= 500 || error.status === 429 || error.code === 'session_not_ready';
  return true;
};

const failureMessage = (error) =>
  error instanceof EveSessionExpiredError
    ? 'The history of this eve session is no longer available (retention expired).'
    : `Lost the connection to the eve agent: ${error.message}`;

/**
 * Reads an eve session's stream in the BACKGROUND, one reader per chat, and turns what happens
 * there into what the chat already knows how to show: persisted items, the busy flag and the
 * approval requests the operator answers.
 *
 * It runs in the background because a turn can wait on a human for hours: nothing may hold a
 * request open for that. The reader stops when the turn parks on a person (`awaiting_input`) or
 * the session settles; whoever sends something next calls `ensure` again, which resumes from the
 * stored cursor.
 *
 * Delivery is at-least-once. The cursor is saved AFTER an event's effects ran, so a crash in
 * between replays that one event; the audit is idempotent (unique index), the persisted items are
 * not, and that window is a few milliseconds wide.
 *
 * @param {object} deps - everything with a side effect, injected so the relay is testable offline
 * @param {object} deps.client - `eveClient`
 * @param {{load: Function, save: Function}} deps.store - per-chat agent, sessionId, cursor, turn
 * @param {{add: Function}} deps.items - persists a chat item and returns it
 * @param {object} deps.emit - the outbound side: assistantMessage, assistantError, busy, activity,
 *   approvalRequested, approvalResolved
 * @param {{recordSafe: Function}} deps.audit
 * @param {(ms: number) => Promise<void>} [deps.sleep]
 */
export class EveRelay {
  constructor({
    client,
    store,
    items,
    emit,
    audit,
    onChildSession = async () => {},
    sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms)),
  }) {
    this.client = client;
    this.store = store;
    this.items = items;
    this.emit = emit;
    this.audit = audit;
    this.onChildSession = onChildSession;
    this.sleep = sleep;
    /** @type {Map<string, {controller: AbortController, again: boolean, promise: Promise<void>}>} */
    this.running = new Map();
  }

  isRunning(chatId) {
    return this.running.has(chatId);
  }

  /**
   * Makes sure a reader is running for the chat. Safe to call whenever something was sent to the
   * session: if one is already reading it will pick the new events up, and if it is about to stop
   * it reads once more.
   * @returns {Promise<void>} settles when the reader stops (tests await it; callers need not)
   */
  ensure(chatId) {
    const existing = this.running.get(chatId);
    if (existing) {
      existing.again = true;
      return existing.promise;
    }
    const entry = { controller: new AbortController(), again: false, promise: null };
    this.running.set(chatId, entry);
    entry.promise = this._loop(chatId, entry);
    return entry.promise;
  }

  stop(chatId) {
    this.running.get(chatId)?.controller.abort();
  }

  stopAll() {
    for (const chatId of this.running.keys()) this.stop(chatId);
  }

  async _loop(chatId, entry) {
    try {
      do {
        entry.again = false;
        await this._read(chatId, entry);
      } while (entry.again && !entry.controller.signal.aborted);
    } catch (error) {
      chatLogger.error(`EveRelay: reader for chat ${chatId} crashed:`, error);
    } finally {
      // Same tick as the `again` check above, no await in between: a `ensure` can never land in a
      // gap where the entry is still registered but nobody is going to read.
      this.running.delete(chatId);
    }
  }

  async _read(chatId, entry) {
    const { signal } = entry.controller;
    const { agent, sessionId, cursor, turn, parentChatId } = await this.store.load(chatId);
    if (!sessionId) throw new Error(`chat ${chatId} has no eve session to read`);

    let state = restoreTurn(turn);
    let index = cursor ?? 0;
    let sinceSave = 0;
    let attempt = 0;
    let published = null;
    const seen = new Set();

    const save = async () => {
      await this.store.save(chatId, { cursor: index, turn: serializeTurn(state) });
      sinceSave = 0;
    };

    // Busy and the activity label follow the state. Nothing is published while the turn has not
    // started: the caller already said "busy" when it sent the message, and flipping it off and on
    // again on the events that precede the turn would flicker the input.
    const publish = () => {
      if (state.phase === TURN_PHASE.IDLE) return;
      const busy = state.phase === TURN_PHASE.RUNNING;
      const activity = activityOf(state);
      const key = `${busy}|${activity.phase}|${activity.label}`;
      if (key === published) return;
      published = key;
      this.emit.busy(chatId, busy);
      this.emit.activity(chatId, activity);
    };

    try {
      while (!isReadComplete(state) && !signal.aborted) {
        let progressed = false;
        let failure = null;

        try {
          for await (const event of this.client.streamEvents(agent, sessionId, { startIndex: index, signal })) {
            progressed = true;
            index++;
            sinceSave++;

            // Position first, identity second: a duplicate is still a place in the stream.
            const id = event.meta?.id;
            if (id) {
              if (seen.has(id)) continue;
              seen.add(id);
            }

            if (NOISE_EVENTS.has(event.type)) {
              if (sinceSave >= SAVE_EVERY) await save();
              continue;
            }

            // Only the child's initial brief comes from Eve; GCS-originated input is already stored.
            if (parentChatId && event.type === 'message.received' && event.data?.sequence === 0) {
              await this.items.initialUserMessage?.(chatId, event.data.message);
            }
            const result = reduceEvent(state, event);
            state = result.state;
            for (const effect of result.effects) await this._apply(chatId, effect);
            publish();

            const changed = result.effects.length > 0 || event.type === 'message.completed';
            if (changed || sinceSave >= SAVE_EVERY || isReadComplete(state)) await save();
            if (isReadComplete(state)) break;
          }
        } catch (error) {
          if (signal.aborted) break;
          failure = error;
        }

        if (signal.aborted || isReadComplete(state)) break;

        // The stream ended or broke before the turn did (eve restarted, the network dropped).
        if (failure && !isTransient(failure)) {
          state = await this._fail(chatId, state, failure, index);
          break;
        }
        attempt = progressed ? 1 : attempt + 1;
        if (attempt > RECONNECT_BACKOFF_MS.length) {
          state = await this._fail(chatId, state, failure ?? new Error('the stream keeps closing'), index);
          break;
        }
        chatLogger.warn(`EveRelay: stream of chat ${chatId} closed mid-turn, reconnecting (attempt ${attempt})`);
        await this.sleep(RECONNECT_BACKOFF_MS[attempt - 1]);
      }
    } finally {
      if (sinceSave > 0 && state.phase !== TURN_PHASE.IDLE) {
        await save().catch((error) =>
          chatLogger.error(`EveRelay: could not save the cursor of chat ${chatId}:`, error)
        );
      }
      // A parked turn is not busy (the operator can still talk, and answer), a stopped reader is not either.
      if (state.phase !== TURN_PHASE.IDLE) {
        this.emit.busy(chatId, false);
        this.emit.activity(chatId, activityOf(state));
      }
    }
  }

  /** The reader cannot continue: close the turn the way a failed one closes, and say why. */
  async _fail(chatId, state, error, index) {
    chatLogger.error(`EveRelay: giving up on chat ${chatId}: ${error.message}`);
    const { state: failed, effects } = reduceEvent(state, {
      type: 'turn.failed',
      data: { error: { message: failureMessage(error) } },
    });
    for (const effect of effects) await this._apply(chatId, effect);
    await this.store.save(chatId, { cursor: index, turn: serializeTurn(failed) });
    return failed;
  }

  async _apply(chatId, effect) {
    switch (effect.kind) {
      case 'child_session':
        await this.onChildSession(chatId, effect);
        break;

      case 'tool_call': {
        const item = await this.items.add(chatId, {
          type: 'function_call',
          name: effect.name,
          arguments: JSON.stringify(effect.input ?? {}),
          call_id: effect.callId,
        });
        this.emit.assistantMessage(item);
        break;
      }

      case 'tool_result': {
        const item = await this.items.add(chatId, {
          type: 'function_call_output',
          name: effect.name,
          call_id: effect.callId,
          output: asText(effect.output),
          ...(effect.isError ? { status: 'error' } : {}),
        });
        this.emit.assistantMessage(item);

        // A gated call: record what became of it. `rejected` means it never ran.
        if (effect.requestId) {
          await this.audit.recordSafe({
            engine: 'eve',
            chatId,
            requestId: effect.requestId,
            callId: effect.callId,
            toolName: effect.name,
            kind: 'executed',
            source: 'eve',
            text: effect.status,
          });
        }
        break;
      }

      case 'input_requested': {
        for (const request of effect.requests) {
          if (request.kind !== 'tool-approval') continue;
          await this.audit.recordSafe({
            engine: 'eve',
            chatId,
            requestId: request.requestId,
            callId: request.action?.callId ?? null,
            toolName: request.toolName,
            kind: 'requested',
            source: 'eve',
            input: request.action?.input ?? null,
            eveEventId: effect.eveEventId,
          });
        }
        this.emit.approvalRequested(
          chatId,
          effect.requests.map((request) => ({ ...request, chatId }))
        );
        break;
      }

      case 'input_resolved': {
        for (const resolution of effect.resolutions) {
          const outcome = toAuditOutcome(resolution.outcome);
          if (resolution.request?.kind !== 'tool-approval' || !outcome) continue;
          await this.audit.recordSafe({
            engine: 'eve',
            chatId,
            requestId: resolution.requestId,
            callId: resolution.request.action?.callId ?? null,
            toolName: resolution.request.toolName,
            kind: 'decided',
            // `derived` = the turn ended with the request still open: nobody decided, the GCS closed it.
            source: effect.derived ? 'system' : 'eve',
            outcome,
            text: resolution.response?.text ?? null,
            eveEventId: effect.eveEventId ?? null,
          });
        }
        this.emit.approvalResolved(
          chatId,
          effect.resolutions.map(({ requestId, outcome }) => ({ requestId, outcome, responderPrincipalId: null }))
        );
        break;
      }

      case 'turn_end': {
        if (effect.phase === TURN_PHASE.COMPLETED && effect.finalText) {
          const item = await this.items.add(chatId, { role: 'assistant', type: 'text', content: effect.finalText });
          this.emit.assistantMessage(item);
        } else if (effect.phase === TURN_PHASE.FAILED) {
          this.emit.assistantError(chatId, effect.error ?? 'The eve agent failed this turn');
        }
        break;
      }

      // 'phase' and 'task' only move the activity label, which `publish` derives from the state.
      default:
        break;
    }
  }
}
