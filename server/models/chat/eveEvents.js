/**
 * Pure state machine over an eve session's event stream. No I/O: `reduceEvent(state, event)`
 * returns the next state plus the effects the caller must perform (persist an item, tell the
 * client, write the audit). `EveRelay` owns everything with side effects.
 *
 * eve's stream is a durable event log, not a request/response: a turn is a SPAN of events, and it
 * can pause in the middle for a human. The previous bridge drained to the first `session.waiting`,
 * which never arrives while a tool approval is open (the turn emits `input.requested`, then
 * `turn.waiting {on:"input"}`, and the stream stays open) — so the reader hung and the chat stayed
 * busy forever. Here a turn ends on `turn.completed|failed|cancelled`, and `turn.waiting` is read
 * for what it says: `on:"input"` is a human, `on:"tasks"` is sub-work and the turn is still running.
 *
 * Shapes below are the ones captured from eve 0.81.1 (stream v26); nothing here is guessed from
 * the docs except where a comment says so.
 */

export const TURN_PHASE = Object.freeze({
  IDLE: 'idle', // no turn open: events seen now belong to a turn that already ended
  RUNNING: 'running',
  AWAITING_INPUT: 'awaiting_input', // parked on a human; the reader stops here
  COMPLETED: 'completed',
  FAILED: 'failed',
  CANCELLED: 'cancelled',
});

const TURN_END_PHASES = new Set([TURN_PHASE.COMPLETED, TURN_PHASE.FAILED, TURN_PHASE.CANCELLED]);

/** Events that only make the stream long: they never change what the operator sees as a message. */
export const NOISE_EVENTS = new Set(['message.appended', 'reasoning.appended', 'action.input.appended']);

/**
 * @typedef {object} EveTurnState
 * @property {string} phase - one of TURN_PHASE
 * @property {?{text: string, finishReason: ?string, stepIndex: ?number}} lastMessage - the last
 *   `message.completed` of the turn. A turn appends one per step (tool-call reasoning first, the
 *   answer last), so the LAST wins; `finishReason` cannot tell them apart (two `stop` in a row
 *   happen when a task fails after the first answer).
 * @property {Array<object>} pending - open input requests, normalised (see normalizeInputRequest)
 * @property {Record<string, string>} tasks - active tasks, taskId → name
 * @property {string[]} hiddenCalls - callIds of calls the operator does not need to see (skill loads)
 * @property {Record<string, string>} approvalCalls - callId → requestId of the calls that asked a human
 * @property {boolean} settled - the session reported it is waiting/completed/failed after a turn end
 */

/** @returns {EveTurnState} */
export const initialTurnState = (overrides = {}) => ({
  phase: TURN_PHASE.IDLE,
  lastMessage: null,
  pending: [],
  tasks: {},
  hiddenCalls: [],
  approvalCalls: {},
  settled: false,
  ...overrides,
});

/** What the operator can answer with, as the UI renders it. */
const normalizeOptions = (options) =>
  (options ?? []).map(({ id, label, description, style }) => ({ id, label: label ?? id, description, style }));

/**
 * eve's `input.requested.data.requests[i]` → the InputRequest the client already renders for the
 * legacy engine (`toolApprovalStore.toInputRequest`). Differences are explicit: no `expiresAt` and
 * no `inputHash` (eve has neither), `options` come from the event.
 */
export const normalizeInputRequest = (request, eventData = {}) => {
  const action = request.action
    ? {
        kind: request.action.kind ?? 'tool-call',
        callId: request.action.callId ?? request.callId ?? null,
        toolName: request.action.toolName ?? request.action.name ?? null,
        input: request.action.input ?? null,
      }
    : null;

  return {
    requestId: request.requestId,
    kind: request.kind ?? 'tool-approval',
    source: 'eve',
    prompt: request.prompt ?? null,
    display: request.display ?? null,
    allowFreeform: Boolean(request.allowFreeform),
    options: normalizeOptions(request.options),
    toolName: action?.toolName ?? null,
    action,
    taskId: request.taskId ?? eventData.taskId ?? null,
  };
};

/** eve's resolution outcomes → the audit vocabulary. `answered` has no approval meaning. */
const AUDIT_OUTCOME = {
  approved: 'approved',
  denied: 'denied',
  cancelled: 'cancelled',
  ignored: 'ignored',
  invalid: 'invalid',
};
export const toAuditOutcome = (outcome) => AUDIT_OUTCOME[outcome] ?? null;

const withPhase = (state, phase, effects) => {
  if (state.phase === phase) return { state, effects };
  return { state: { ...state, phase }, effects: [...effects, { kind: 'phase', phase }] };
};

const endTurn = (state, phase, extra = {}) => {
  const effects = [];

  // Whatever is still open when the turn ends is over: cancelled or failed turns drop their
  // approvals, and the card must not outlive them.
  if (state.pending.length > 0) {
    effects.push({
      kind: 'input_resolved',
      derived: true,
      resolutions: state.pending.map((request) => ({
        request,
        requestId: request.requestId,
        outcome: 'cancelled',
        response: null,
      })),
    });
  }

  effects.push({
    kind: 'turn_end',
    phase,
    finalText: phase === TURN_PHASE.COMPLETED ? (state.lastMessage?.text ?? '') : '',
    error: extra.error ?? null,
  });

  return {
    state: { ...state, phase, pending: [], tasks: {} },
    effects: [{ kind: 'phase', phase }, ...effects],
  };
};

const taskLabel = (data) => data.name ?? data.taskId ?? 'task';

/**
 * Applies one event.
 * @param {EveTurnState} state
 * @param {{type: string, data?: object, meta?: {id?: string}}} event
 * @returns {{state: EveTurnState, effects: Array<object>}}
 */
export function reduceEvent(state, event) {
  const data = event.data ?? {};
  const eveEventId = event.meta?.id ?? null;

  switch (event.type) {
    case 'message.received':
    case 'turn.started': {
      // A new turn: forget the last one's text, or a turn that says nothing would repeat it.
      const next = { ...state, lastMessage: null, settled: false };
      return withPhase(next, TURN_PHASE.RUNNING, []);
    }

    case 'message.completed': {
      if (typeof data.message !== 'string') return { state, effects: [] };
      const lastMessage = {
        text: data.message,
        finishReason: data.finishReason ?? null,
        stepIndex: data.stepIndex ?? null,
      };
      return { state: { ...state, lastMessage }, effects: [] };
    }

    case 'actions.requested': {
      const effects = [];
      const hiddenCalls = [...state.hiddenCalls];
      for (const action of data.actions ?? []) {
        // Loading a skill is the agent reading its own manual: not something to show the operator.
        if (action.kind === 'load-skill') {
          hiddenCalls.push(action.callId);
          continue;
        }
        effects.push({
          kind: 'tool_call',
          callId: action.callId,
          // A plain tool call carries `toolName`, a subagent call carries `name` (captured).
          name: action.toolName ?? action.name ?? 'tool',
          input: action.input ?? {},
          isSubagent: action.kind === 'subagent-call',
        });
      }
      return { state: { ...state, hiddenCalls }, effects };
    }

    case 'action.result': {
      const result = data.result ?? {};
      if (state.hiddenCalls.includes(result.callId)) return { state, effects: [] };
      // `status` is eve's verdict (`completed`, `failed`, `rejected`); a rejected call never ran.
      const status = data.status ?? 'completed';
      return {
        state,
        effects: [
          {
            kind: 'tool_result',
            callId: result.callId,
            requestId: state.approvalCalls[result.callId] ?? null,
            name: result.toolName ?? null,
            output: result.output ?? result.error ?? data.error ?? null,
            status,
            isError: status !== 'completed' || Boolean(result.isError ?? data.error),
          },
        ],
      };
    }

    case 'task.started': {
      const tasks = { ...state.tasks, [data.taskId]: taskLabel(data) };
      return {
        state: { ...state, tasks },
        effects: [
          { kind: 'task', status: 'started', taskId: data.taskId, callId: data.callId ?? null, name: taskLabel(data) },
        ],
      };
    }

    case 'task.settled': {
      const { [data.taskId]: _done, ...tasks } = state.tasks;
      return {
        state: { ...state, tasks },
        effects: [
          {
            kind: 'task',
            status: data.status ?? 'completed',
            taskId: data.taskId,
            callId: data.callId ?? null,
            name: taskLabel(data),
            output: data.output ?? null,
            error: data.error?.message ?? null,
          },
        ],
      };
    }

    case 'input.requested': {
      const requests = (data.requests ?? []).map((request) => normalizeInputRequest(request, data));
      if (requests.length === 0) return { state, effects: [] };
      const known = new Set(state.pending.map((request) => request.requestId));
      const fresh = requests.filter((request) => !known.has(request.requestId));
      // callId → requestId, so the result of the call can be tied back to the decision on it.
      const approvalCalls = { ...state.approvalCalls };
      for (const request of fresh) {
        if (request.action?.callId) approvalCalls[request.action.callId] = request.requestId;
      }
      return {
        state: { ...state, pending: [...state.pending, ...fresh], approvalCalls },
        effects: fresh.length ? [{ kind: 'input_requested', requests: fresh, eveEventId }] : [],
      };
    }

    case 'input.resolved': {
      const resolutions = (data.resolutions ?? []).map((resolution) => ({
        request: state.pending.find((request) => request.requestId === resolution.requestId) ?? null,
        requestId: resolution.requestId,
        outcome: resolution.outcome,
        response: resolution.response ?? null,
      }));
      const resolved = new Set(resolutions.map((resolution) => resolution.requestId));
      const next = { ...state, pending: state.pending.filter((request) => !resolved.has(request.requestId)) };
      const effects = resolutions.length ? [{ kind: 'input_resolved', resolutions, eveEventId }] : [];
      // The turn was parked on this answer: with nothing left open it is running again.
      if (next.phase === TURN_PHASE.AWAITING_INPUT && next.pending.length === 0) {
        return withPhase(next, TURN_PHASE.RUNNING, effects);
      }
      return { state: next, effects };
    }

    case 'turn.waiting': {
      // `on:"tasks"` is sub-work in flight, the turn is still running. Only a human parks it, and
      // only if there really is something open: a replayed `turn.waiting` whose request was already
      // resolved must not stop the reader.
      if (data.on === 'input' && state.pending.length > 0) {
        return withPhase(state, TURN_PHASE.AWAITING_INPUT, []);
      }
      return { state, effects: [] };
    }

    case 'turn.completed':
      return endTurn(state, TURN_PHASE.COMPLETED);

    case 'turn.cancelled':
      return endTurn(state, TURN_PHASE.CANCELLED);

    case 'turn.failed':
      // `terminal:false` would mean eve will retry the turn itself.
      if (data.terminal === false) return { state, effects: [] };
      return endTurn(state, TURN_PHASE.FAILED, { error: data.error?.message ?? 'The eve agent failed this turn' });

    case 'session.failed': {
      if (TURN_END_PHASES.has(state.phase)) return { state: { ...state, settled: true }, effects: [] };
      const ended = endTurn(state, TURN_PHASE.FAILED, { error: data.error?.message ?? 'The eve session failed' });
      return { state: { ...ended.state, settled: true }, effects: ended.effects };
    }

    case 'session.waiting':
    case 'session.completed': {
      // Before any turn started in this reader it is the tail of a turn that already ended (the
      // reader begins at a stored cursor): not ours to act on.
      if (state.phase === TURN_PHASE.IDLE) return { state, effects: [] };
      if (TURN_END_PHASES.has(state.phase)) return { state: { ...state, settled: true }, effects: [] };
      // The session is waiting for the next user message, so the turn is over even though no
      // `turn.*` event said so. Not seen in practice; kept so a missing event cannot hang the chat.
      if (state.phase === TURN_PHASE.RUNNING) {
        const ended = endTurn(state, TURN_PHASE.COMPLETED);
        return { state: { ...ended.state, settled: true }, effects: ended.effects };
      }
      return { state, effects: [] };
    }

    default:
      return { state, effects: [] };
  }
}

/**
 * What the operator should see while a turn is open. `planning` is a running turn that is waiting
 * on a task (a plan can take minutes: without it the UI is a spinner with no explanation).
 * @param {EveTurnState} state
 * @returns {{phase: 'idle'|'running'|'planning'|'awaiting_input', label: ?string}}
 */
export function activityOf(state) {
  if (state.phase === TURN_PHASE.AWAITING_INPUT) return { phase: 'awaiting_input', label: null };
  if (state.phase !== TURN_PHASE.RUNNING) return { phase: 'idle', label: null };
  const tasks = Object.values(state.tasks);
  if (tasks.length > 0) return { phase: 'planning', label: tasks[0] };
  return { phase: 'running', label: null };
}

/** True when the reader has nothing more to do until something is sent to the session. */
export const isReadComplete = (state) => state.phase === TURN_PHASE.AWAITING_INPUT || state.settled;
