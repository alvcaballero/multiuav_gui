import { randomUUID } from 'node:crypto';
import { MCPclient } from './mcpClient.js';
import { LLMFactory } from './handlers/llmFactory.js';
import { chatLogger } from '../../common/logger.js';
import { LLM, MCPenable, TOOL_APPROVAL_ENFORCE } from '../../config/config.js';
import {
  requiresApproval,
  assertApprovedInputMatches,
  freezeToolCall,
  ToolApprovalRequiredError,
} from './toolApproval.js';
import { TurnContext } from './turnContext.js';
import { ChatHistoryManager } from './chatHistoryManager.js';
import {
  emitAssistantError,
  emitAssistantMessage,
  emitChatBusy,
  emitToolApprovalRequested,
  emitToolApprovalResolved,
} from './chatEvents.js';
import { ToolApprovalStore, toInputRequest } from './toolApprovalStore.js';
import { getContextParams, removeSubAgent } from './subAgentRegistry.js';
import { forceFinishItem, RETRY_AFTER_TOOL_ERROR_MESSAGE } from './handlers/baseLLMhandler.js';

let mcpClient = null;
let llmHandler = null;

const maxIterations = 25; // Prevenir loops infinitos
const maxIterations_planner = 18; // Prevenir loops infinitos

// Per-chatId mutex: ensures only one processMessage runs at a time per chat.
// Concurrent requests for the same chatId queue behind the active one.
const chatLocks = new Map();

export class MessageOrchestrator {
  static initializeLLMProvider(provider, apiKey, options = {}) {
    if (LLM && !llmHandler) {
      const { model = '', ...handlerOptions } = options;
      llmHandler = LLMFactory.createHandler(provider, apiKey, model || undefined, handlerOptions);
      llmHandler
        .initialize()
        .then(() => {
          chatLogger.info('LLM Handler initialized successfully.');
        })
        .catch((error) => {
          chatLogger.error('Error initializing LLM Handler:', error);
          llmHandler = null;
        });
      chatLogger.info('LLM Provider initialized:', { provider });
    }
    if (MCPenable && !mcpClient) {
      mcpClient = new MCPclient();
      mcpClient
        .connect()
        .then(() => {
          chatLogger.info('MCP Client connected successfully.');
        })
        .catch((error) => {
          chatLogger.error('Error connecting MCP Client:', error);
        });
    } else if (!MCPenable) {
      chatLogger.info('MCP is disabled, using only LLM provider.');
    }
  }

  /**
   * Verifica si el orquestador está listo para procesar mensajes
   * @returns {boolean}
   */

  static isReady() {
    return llmHandler !== null && llmHandler !== undefined && llmHandler.initialized === true;
  }

  /**
   * Emits EventBus event for assistant messages (WebSocket broadcast to clients)
   * @param {object} chatItem - The chat item to potentially emit
   */
  static emitAssistantMessage(chatItem) {
    emitAssistantMessage(chatItem);
  }

  /**
   * Normalizes whatever a caller passes as the turn input into the canonical
   * `{kind, ...}` shape. Plain strings and multipart arrays are user messages,
   * which keeps every existing `processMessage(chatId, "text")` call working.
   *
   * @param {string|Array|object} input
   * @returns {{kind: string}} Canonical turn input
   */
  static _normalizeInput(input) {
    if (typeof input === 'string' || Array.isArray(input)) {
      return { kind: 'user', content: input };
    }
    if (input && typeof input === 'object' && input.kind) return input;
    throw new Error('processMessage: input must be a string, a content array, or a {kind} object');
  }

  /**
   * Procesa un turno con mutex por chatId.
   * Concurrent requests for the same chatId queue sequentially.
   *
   * The lock is held for the WHOLE turn, tool loop included. The turn itself is
   * fire-and-forget past the first LLM response (the caller gets that response
   * while the loop keeps streaming over the EventBus), so the lock cannot be
   * released in this function's `finally` — that would free the chat while the
   * loop is still writing to it. `_runTurn` releases it when the recursion ends.
   *
   * @param {string} chatId - ID de la conversación
   * @param {string|Array|object} input - User message, or a `{kind, ...}` turn input
   * @param {Object} options - Opciones adicionales
   * @param {Array<string>} options.allowedTools - Lista de herramientas permitidas (null = todas)
   * @returns {Promise<Object>} Respuesta final
   */
  static async processMessage(chatId, input, options = {}) {
    const turnInput = this._normalizeInput(input);

    const release = await this._acquireChatLock(chatId);

    try {
      return await this._startTurn(chatId, turnInput, options, release);
    } catch (error) {
      release();
      throw error;
    }
  }

  /**
   * Takes the per-chat mutex, queueing behind whoever holds it.
   * @returns {Promise<Function>} `release`, which the caller MUST eventually call
   */
  static async _acquireChatLock(chatId) {
    const prev = chatLocks.get(chatId) || Promise.resolve();
    let resolve;
    const lock = new Promise((r) => {
      resolve = r;
    });
    chatLocks.set(chatId, lock);

    // Idempotent: the resume path has both a `finally(release)` deep in the
    // recursion and its own error handler, and neither can know about the other.
    let released = false;
    const release = () => {
      if (released) return;
      released = true;
      resolve();
      // Clean up if no one else is queued behind us
      if (chatLocks.get(chatId) === lock) {
        chatLocks.delete(chatId);
      }
      emitChatBusy(chatId, false);
    };

    await prev;
    emitChatBusy(chatId, true);

    return release;
  }

  /**
   * Internal: runs the first turn (called under chatId mutex).
   *
   * Orchestration only: ask for context, run the turn, hand the caller the first
   * response. If the model asked for tools, `_runTurn` recursed into the loop
   * before returning here, and `release` has already fired.
   */
  static async _startTurn(chatId, turnInput, options = {}, release = () => {}) {
    // Groups every LLM request triggered by this input (initial call + tool loop
    // iterations) so the real cost of one turn is a single GROUP BY away.
    const turnId = randomUUID();
    this._logIncomingMessage(chatId, turnInput);

    // A subagent answer is appended to a conversation the provider's session
    // never saw, so the session is dropped to force the full-history path.
    if (turnInput.kind === 'subagent_result') {
      await ChatHistoryManager.clearSession(chatId);
    }

    const ctx = await TurnContext.build(chatId, {
      allowedTools: options.allowedTools ?? null,
      getTools: (allowed) => this.getToolsForProvider(allowed),
      llmHandler,
    });

    try {
      const result = await this._runTurn(chatId, turnInput, ctx, {
        turnId,
        phase: 'initial',
        iteration: 0,
        release,
      });

      chatLogger.info('✓ Procesamiento completado para chat:', chatId);
      return llmHandler.normalizeResponse(result.raw);
    } catch (error) {
      chatLogger.error('Error procesando mensaje:', error);

      // Let the handler handle session-related errors
      await llmHandler.handleSessionError(chatId, error, ctx.persistence);
      emitAssistantError(chatId, error.userMessage || `Error: ${error.message}`);

      throw error;
    }
  }

  /**
   * One turn: persist the input, call the LLM, persist the output — then, if the
   * model asked for tools, run them and recurse with their results.
   *
   * This single function replaces the old `_processMessage` / `continueAfterTools`
   * pair, which were the same three steps with different inputs. What varies is
   * ONLY how the input reaches the provider, and that lives in `_applyInput`.
   *
   * The recursion carries `iteration` as a parameter rather than tracking it per
   * chat: the cap protects ONE turn from running away, and a turn already has an
   * identity (`turnId`). A fresh user message is a fresh turn, so it starts at 0
   * with nothing to reset.
   *
   * @param {string} chatId
   * @param {object} turnInput - `{kind, ...}` canonical input
   * @param {TurnContext} ctx
   * @param {object} meta - `{turnId, phase, iteration, release}`
   * @returns {Promise<{output: Array, responseId: string, raw: object}>}
   */
  static async _runTurn(chatId, turnInput, ctx, meta) {
    const { turnId, phase, iteration, release } = meta;

    // Read fresh HERE — before `_applyInput` persists this turn's own input below —
    // so it never contains what this turn is about to write. That write travels
    // separately via providerInput; reading it back from a later DB read would
    // send it to the provider twice. (With a live session the provider keeps the
    // conversation itself, so most handlers ignore this in the happy path — it
    // only matters as their session-error fallback.)
    const history = await ChatHistoryManager.loadHistory(chatId);

    const providerInput = await this._applyInput(chatId, turnInput);

    // forceFinish rides inside providerInput.items instead of a separate flag —
    // each handler picks the 'directive' item out and inserts it wherever its
    // API requires. Only a tool_output turn ever sets forceFinish (see
    // _continueWithTools), so providerInput is always the 'tool_output' shape here.
    if (turnInput.forceFinish && providerInput?.type === 'tool_output') {
      providerInput.items = [...providerInput.items, forceFinishItem()];
    }

    const result = await llmHandler.processMessage(providerInput, ctx.tools, history, {
      sessionId: ctx.sessionId,
      instructions: ctx.systemInstructions,
      agent: ctx.agent,
    });

    const { output, responseId } = await this._persistTurnResult(chatId, result, ctx, {
      turnId,
      phase,
      iteration,
    });

    const hasToolCalls = output.some((res) => res.type === 'function_call' || res.type === 'tool_call');
    if (hasToolCalls) {
      // Fire-and-forget from here on: the caller gets the first response while the
      // loop keeps going and streams over the EventBus. The lock stays held until
      // the recursion bottoms out, so a queued message cannot interleave with it.
      this._continueWithTools(chatId, output, ctx, { turnId, iteration }).finally(release);
      return { output, responseId, raw: result };
    }

    // A provider can fail to emit a well-formed tool call (e.g. Gemini's
    // MALFORMED_FUNCTION_CALL) yet still flag the turn as recoverable. There is
    // no call_id to answer, so this can't join the tool loop above — instead it
    // re-enters the same way a subagent result does: a synthetic message that
    // gives the model another turn to retry on its own, without waiting on the user.
    const isRetryable = output.some((res) => res.retryable);
    const maxIter = ctx.agent?.capability === 'high' ? maxIterations_planner : maxIterations;
    if (isRetryable && iteration + 1 < maxIter) {
      this._continueAsRetry(chatId, ctx, { turnId, iteration }).finally(release);
      return { output, responseId, raw: result };
    }

    release();
    return { output, responseId, raw: result };
  }

  /**
   * Re-enters the turn after a provider-side recoverable failure (no tool call
   * to answer). Mirrors `_continueWithTools`'s recursion shape but goes through
   * `_applyInput`'s `system_message` path since there is no tool result to feed
   * and this isn't a subagent's doing either.
   */
  static async _continueAsRetry(chatId, ctx, { turnId, iteration }) {
    const next = iteration + 1;
    chatLogger.debug(`[ToolLoop: ${chatId}] Iteration ${next} - Retrying after recoverable provider error...`);

    try {
      await this._runTurn(
        chatId,
        { kind: 'system_message', content: RETRY_AFTER_TOOL_ERROR_MESSAGE },
        ctx,
        { turnId, phase: 'tool_loop', iteration: next, release: () => {} }
      );
    } catch (error) {
      chatLogger.error(`[ToolLoop: ${chatId}] Error retrying after provider error:`, error);
      emitAssistantError(chatId, `Error retrying tool call: ${error.message}`);
    }
  }

  /**
   * Executes the tool calls in `output` and recurses with their results.
   * Split out of `_runTurn` so the fire-and-forget boundary is explicit: this is
   * the part that outlives the caller's turn.
   */
  static async _continueWithTools(chatId, output, ctx, { turnId, iteration }) {
    const maxIter = ctx.agent?.capability === 'high' ? maxIterations_planner : maxIterations;
    const next = iteration + 1;
    const isLastIteration = next >= maxIter;

    if (isLastIteration) {
      chatLogger.warn(`[ToolLoop: ${chatId}] Maximum iterations reached - forcing final response`);
    }
    chatLogger.debug(`[ToolLoop: ${chatId}] Iteration ${next} - Processing tools...`);

    try {
      // A gated call parks the WHOLE batch: running the read-only calls now and
      // the flight call minutes later would hand the operator a decision based on
      // telemetry that no longer holds.
      if (await this._parkForApproval(chatId, output, ctx, { turnId, iteration: next })) return;

      const results = await this.executeToolCalls(output, chatId);

      await this._runTurn(
        chatId,
        { kind: 'tool_output', results, forceFinish: isLastIteration },
        ctx,
        // The last iteration must not recurse again: `release` is a no-op here
        // because THIS call's `.finally(release)` already owns it.
        { turnId, phase: 'tool_loop', iteration: next, release: () => {} }
      );
    } catch (error) {
      chatLogger.error(`[ToolLoop: ${chatId}] Error in tool calls loop:`, error);
      emitAssistantError(chatId, `Error processing tool results: ${error.message}`);
    }
  }

  /**
   * Freezes every gated call in the batch and parks the turn.
   *
   * Returns true when the turn parked. The chat lock is released by the caller's
   * `.finally(release)` as usual, so the operator can still talk to the chat —
   * and, more to the point, still answer.
   */
  static async _parkForApproval(chatId, output, ctx, { turnId, iteration }) {
    if (!TOOL_APPROVAL_ENFORCE) return false;

    const toolCalls = output.filter((r) => r.type === 'function_call' || r.type === 'tool_call');
    const contextParams = getContextParams(chatId);
    const hasContext = Object.keys(contextParams).length > 0;

    const gated = toolCalls.filter((call) => requiresApproval(call.name));
    if (gated.length === 0) return false;

    const resumeContext = { allowedTools: ctx.allowedTools ?? null, agent: ctx.agent?.name ?? null, toolCalls };

    const requests = [];
    for (const call of gated) {
      const args = JSON.parse(call.arguments);
      const frozen = freezeToolCall({
        callId: call.call_id,
        toolName: call.name,
        input: hasContext ? { ...args, ...contextParams } : args,
      });
      const row = await ToolApprovalStore.createPending({ chatId, turnId, iteration, frozen, resumeContext });
      requests.push(toInputRequest(row));
    }

    emitToolApprovalRequested(chatId, requests);
    chatLogger.warn(`[ToolApproval] Turn ${turnId} parked on ${requests.length} pending approval(s)`);
    return true;
  }

  /**
   * Persists a turn's input and returns what the provider needs for it.
   *
   * Every kind writes its own input to history exactly once — that symmetry is
   * what removed the old `skipPersist` flag, which existed only because the
   * subagent path wrote its message somewhere else first.
   *
   * @returns {Promise<?{type: 'message', content: *}|{type: 'tool_output', items: Array}>} What
   *   `llmHandler.processMessage` receives as its first argument. `null` means "nothing new to
   *   send — continue purely from the persisted history".
   */
  static async _applyInput(chatId, turnInput) {
    switch (turnInput.kind) {
      case 'user':
        await ChatHistoryManager.addMessage(chatId, 'user', { role: 'user', content: turnInput.content });
        // The history _runTurn read is from BEFORE this message was persisted
        // above, so it serves as full context on the no-session fallback path.
        return { type: 'message', content: turnInput.content };

      case 'subagent_result': {
        const chatItem = await ChatHistoryManager.addMessage(chatId, 'subagent', turnInput.message);
        this.emitAssistantMessage(chatItem);
        // Sent as its own turn item (like tool_output), not replayed from
        // history: the parent's tool pair closed long ago, so there is no
        // pending call for it to answer — it's a fresh user-role message.
        return { type: 'subagent_result', message: turnInput.message };
      }

      case 'system_message': {
        // Orchestrator-authored nudge, not from a user or a subagent — e.g. a
        // provider failed to emit a parseable tool call and this re-enters the
        // turn on its own. Persisted with its own `type` so the client can hide
        // it from the transcript (see ChatMessages.jsx), same idea as
        // subagent_result never showing as a user bubble; sent to the provider
        // as a plain 'message' (every handler already speaks that type), with
        // the directive marker carrying the "system talking" meaning the
        // transport role cannot.
        const chatItem = await ChatHistoryManager.addMessage(chatId, 'system', {
          type: 'system_directive',
          role: 'system',
          content: turnInput.content,
        });
        this.emitAssistantMessage(chatItem);
        return { type: 'message', content: turnInput.content };
      }

      case 'tool_output': {
        for (const res of turnInput.results) {
          const chatItem = await ChatHistoryManager.addMessage(chatId, 'assistant', res);
          this.emitAssistantMessage(chatItem);
        }
        return { type: 'tool_output', items: turnInput.results };
      }

      default:
        throw new Error(`_applyInput: unknown turn input kind "${turnInput.kind}"`);
    }
  }

  /**
   * Logs the incoming turn input, collapsing multipart content to a summary
   * so a base64 image never lands in the logs.
   */
  static _logIncomingMessage(chatId, turnInput) {
    const { kind, content } = turnInput;
    let preview;

    if (kind !== 'user') {
      preview = `[${kind}]`;
    } else if (Array.isArray(content)) {
      preview = `[${content.length} blocks: ${content.map((b) => b.type).join(', ')}]`;
    } else {
      preview = `"${String(content).substring(0, 40)}${String(content).length > 40 ? '...' : ''}"`;
    }

    chatLogger.info(`📨 Chat: ${chatId} | Mensaje: ${preview}`);
  }

  /**
   * Records usage, reconciles session state and persists every output part.
   *
   * Deciding what happens NEXT (tool loop or not) belongs to `_runTurn`; this
   * one only writes down what came back.
   *
   * @returns {Promise<{output: Array, responseId: string}>}
   */
  static async _persistTurnResult(chatId, result, ctx, { turnId, phase, iteration }) {
    const { output, responseId, model, sessionCleared, usage } = result;
    chatLogger.info(`✓ Parsed ${output.length} output parts from LLM response`);

    await ChatHistoryManager.recordUsage({
      chatId,
      turnId,
      usage,
      responseId,
      provider: llmHandler.getProviderName(),
      model,
      agent: ctx.agent.name,
      phase,
      iteration,
    });

    // Let the handler recover from session errors (e.g., recreate expired conversation)
    if (sessionCleared) {
      await llmHandler.handleSessionError(chatId, result, ctx.persistence);
    }
    // Store metadata for this chat
    else if (responseId) {
      await ChatHistoryManager.updateChatMetadata(chatId, {
        responseId,
        provider: llmHandler.getProviderName(),
        model,
      });
    }

    for (const res of output) {
      const chatItem = await ChatHistoryManager.addMessage(chatId, 'assistant', res, responseId);
      this.emitAssistantMessage(chatItem);
    }

    return { output, responseId };
  }

  /**
   * Ejecuta todas las llamadas a herramientas solicitadas por el LLM
   *
   * @param {Array} toolCalls
   * @param {string} chatId
   * @param {object} [options]
   * @param {Map<string, object>} [options.approvals] - Frozen approvals by call_id
   */
  static async executeToolCalls(toolCalls, chatId, { approvals = null } = {}) {
    const results = [];
    const contextParams = getContextParams(chatId);
    const hasContext = Object.keys(contextParams).length > 0;

    for (const toolCall of toolCalls) {
      if (toolCall.type == 'function_call' || toolCall.type == 'tool_call') {
        const approval = approvals?.get(toolCall.call_id) ?? null;

        // An approved call replays the input the operator actually saw. Resolving
        // it again here would re-derive a value nobody authorised.
        const call = approval?.status === 'approved' ? { ...toolCall, arguments: JSON.stringify(approval.input) } : toolCall;

        // Handlers turn any throw from the executor into a generic error result,
        // so the denial is captured here instead: a refused call never ran, and
        // must not be reported as a tool that failed.
        let denial = null;

        const result = await llmHandler.handleToolCall(call, async (name, args) => {
          // Fixed context params win over whatever the LLM passes, so it can't
          // override a subagent's injected context even if it hallucinates the same key.
          const input = approval ? args : hasContext ? { ...args, ...contextParams } : args;

          denial = await this._denyUnapprovedToolCall({ callId: toolCall.call_id, name, input, approval });
          if (denial) throw new Error(denial.message);

          return await mcpClient.executeTool(name, input);
        });

        results.push(denial ? { ...result, status: 'rejected', error: denial } : result);
      }
    }

    return results;
  }

  /**
   * The approval gate. Returns a denial descriptor, or null to let the call run.
   *
   * `input` must be the exact value about to reach the tool, because that is what
   * the operator had to have approved.
   */
  static async _denyUnapprovedToolCall({ callId, name, input, approval }) {
    if (!TOOL_APPROVAL_ENFORCE || !requiresApproval(name)) return null;

    try {
      if (!approval) throw new ToolApprovalRequiredError({ callId, toolName: name });

      if (approval.status === 'denied') {
        return {
          code: 'tool_approval_denied',
          message: `"${name}" was denied by the operator${approval.denyReason ? `: ${approval.denyReason}` : ''}`,
        };
      }

      if (approval.status !== 'approved') throw new ToolApprovalRequiredError({ callId, toolName: name });

      assertApprovedInputMatches(approval, input);

      // Claimed once and only once: a retried response or an overlapping resume
      // must not fly the mission twice.
      if (!(await ToolApprovalStore.claimExecution(approval.requestId))) {
        return { code: 'tool_approval_already_executed', message: `"${name}" already ran for call ${callId}` };
      }

      return null;
    } catch (error) {
      chatLogger.error(`[ToolApproval] Refused ${name} (call ${callId}): ${error.message}`);
      return { code: error.code, message: error.message };
    }
  }

  /**
   * Ages out approvals nobody answered, then unblocks the turns they parked.
   *
   * Resuming matters as much as expiring: a parked turn has a `function_call` in
   * history with no result, and providers reject a conversation where a tool call
   * is left unanswered. The resumed turn produces a rejected result for each
   * expired call, which both keeps history well-formed and tells the model — in
   * the only vocabulary it has — that nothing was authorised.
   */
  static async sweepExpiredApprovals() {
    const expired = await ToolApprovalStore.listExpiredUnswept();
    if (expired.length === 0) return 0;

    await ToolApprovalStore.expireStale();

    for (const { chatId, turnId } of ToolApprovalStore.distinctTurns(expired)) {
      chatLogger.warn(`[ToolApproval] Turn ${turnId} resumed with expired approvals — nothing was authorised`);
      await this._resumeParkedTurn(chatId, turnId);
    }

    return expired.length;
  }

  /**
   * Pending approvals for a chat, as InputRequest shapes.
   *
   * There is no client-side store behind this: the requests live in the DB, so a
   * reload re-reads them and a parked approval is still there, still answerable.
   */
  static async getPendingApprovals(chatId) {
    const rows = await ToolApprovalStore.listPending(chatId);
    return rows.map(toInputRequest);
  }

  /**
   * Answers one or more pending approvals and, once none are left pending for
   * that turn, resumes it.
   *
   * ONLY a structured answer decides. Free text never approves anything: with
   * several requests open there is no way to know which one "yes, go ahead"
   * refers to, and guessing wrong arms an aircraft.
   *
   * @param {string} chatId
   * @param {Array<{requestId: string, optionId: string, text?: string}>} responses
   * @param {object} [options]
   * @param {string} [options.responderPrincipalId] - WHO is answering
   * @returns {Promise<{resolutions: Array, stale: Array}>}
   */
  static async respondToApproval(chatId, responses, { responderPrincipalId = null } = {}) {
    const resolutions = [];
    const stale = [];

    for (const { requestId, optionId, text } of responses ?? []) {
      if (optionId !== 'approve' && optionId !== 'deny') {
        stale.push({ requestId, reason: 'invalid_option' });
        continue;
      }

      const row = await ToolApprovalStore.resolve(requestId, {
        outcome: optionId === 'approve' ? 'approved' : 'denied',
        responderPrincipalId,
        denyReason: optionId === 'deny' ? (text ?? null) : null,
      });

      // Null means it was already answered, cancelled or aged out. A stale answer
      // NEVER re-opens the decision — the model has to ask again.
      if (!row) {
        stale.push({ requestId, reason: 'stale' });
        continue;
      }

      resolutions.push({
        requestId,
        callId: row.callId,
        toolName: row.toolName,
        outcome: row.status,
        responderPrincipalId,
        turnId: row.turnId,
      });
    }

    emitToolApprovalResolved(chatId, resolutions);

    for (const turnId of new Set(resolutions.map((r) => r.turnId))) {
      await this._resumeParkedTurn(chatId, turnId);
    }

    return { resolutions, stale };
  }

  /**
   * Resumes a turn once every approval it parked on has an answer.
   *
   * The model is NOT re-consulted: the approved calls run the frozen input, their
   * results are appended, and only then does the loop continue. A model asked
   * twice can answer twice, and the second answer was never approved.
   */
  static async _resumeParkedTurn(chatId, turnId) {
    if ((await ToolApprovalStore.listPendingForTurn(chatId, turnId)).length > 0) return;

    const decided = await ToolApprovalStore.listForTurn(chatId, turnId);
    if (decided.length === 0) return;

    const resumeContext = decided.find((row) => row.resumeContext)?.resumeContext;
    if (!resumeContext?.toolCalls) {
      chatLogger.error(`[ToolApproval] Cannot resume turn ${turnId}: no resumeContext persisted`);
      return;
    }

    const release = await this._acquireChatLock(chatId);

    try {
      const ctx = await TurnContext.build(chatId, {
        allowedTools: resumeContext.allowedTools ?? null,
        getTools: (allowed) => this.getToolsForProvider(allowed),
        llmHandler,
      });

      const approvals = new Map(decided.map((row) => [row.callId, row]));
      const results = await this.executeToolCalls(resumeContext.toolCalls, chatId, { approvals });

      chatLogger.info(`[ToolApproval] Resuming turn ${turnId} on chat ${chatId}`);

      await this._runTurn(chatId, { kind: 'tool_output', results }, ctx, {
        turnId,
        phase: 'tool_loop',
        iteration: decided[0].iteration,
        release,
      });
    } catch (error) {
      release();
      chatLogger.error(`[ToolApproval] Error resuming turn ${turnId}:`, error);
      emitAssistantError(chatId, `Error resuming after approval: ${error.message}`);
    }
  }

  /**
   * Obtiene las herramientas en el formato correcto para el proveedor actual
   * Filtra las herramientas basándose en allowedTools si se especifica
   * @param {Array<string>|null} allowedTools - Lista de herramientas permitidas (null = todas)
   * @returns {Array} Lista de herramientas (filtradas si corresponde)
   */
  static getToolsForProvider(allowedTools = null) {
    if (!mcpClient || !mcpClient.isReady()) {
      chatLogger.debug('No MCP tools available');
      return [];
    }

    const tools = mcpClient.getTools();

    if (!tools || tools.length === 0) {
      chatLogger.debug('No MCP tools available');
      return [];
    }

    // Si allowedTools es null o undefined, devolver todas las herramientas
    if (allowedTools === null || allowedTools === undefined) {
      return tools;
    }

    // Si allowedTools es un array vacío, no devolver herramientas (forzar respuesta de texto)
    if (Array.isArray(allowedTools) && allowedTools.length === 0) {
      chatLogger.debug('No tools allowed (empty allowedTools array)');
      return [];
    }

    // Filtrar herramientas basándose en allowedTools
    const filteredTools = tools.filter((tool) => allowedTools.includes(tool.name));
    chatLogger.debug(`Filtered tools: ${filteredTools.map((t) => t.name).join(', ')} (from ${tools.length} total)`);
    return filteredTools;
  }

  /**
   * Obtiene una página del historial de conversación para un chat específico,
   * la más reciente por defecto o la anterior a `before` (paginación por cursor).
   * @param {string} chatId - ID del chat
   * @param {object} options
   * @param {number} options.limit - Tamaño de página (default 100)
   * @param {string|null} options.before - Cursor ISO timestamp; trae mensajes estrictamente anteriores
   * @returns {Promise<{messages: Array, hasMore: boolean}>}
   */
  static async getHistory(chatId, { limit = 100, before = null } = {}) {
    try {
      // Fetch one extra row to know whether older messages remain, then drop it.
      const rows = await ChatHistoryManager.loadHistory(chatId, {
        all: true,
        limit: limit + 1,
        before,
        order: 'DESC',
      });
      const hasMore = rows.length > limit;
      return { messages: hasMore ? rows.slice(1) : rows, hasMore };
    } catch (error) {
      chatLogger.error('Error loading history from DB:', error);
      return { messages: [], hasMore: false };
    }
  }

  /**
   * Token usage of a chat: per-LLM-call detail, per-turn breakdown and totals.
   * One turn = one user message, which can span many LLM requests
   * (the initial call plus every tool loop iteration).
   * @param {string} chatId
   * @returns {Promise<{totals: object, turns: Array, requests: Array}>}
   */
  static async getUsage(chatId) {
    return ChatHistoryManager.getUsageForChat(chatId);
  }

  /**
   * Lista todos los chats disponibles
   */
  static async listChats() {
    try {
      const dbChats = await ChatHistoryManager.getAllChats();
      const chats = [];

      for (const dbChat of dbChats) {
        const messageCount = await ChatHistoryManager.getMessageCount(dbChat.id);
        chats.push({
          id: dbChat.id,
          name: dbChat.name,
          messageCount,
          lastUpdated: dbChat.updatedAt,
          createdAt: dbChat.createdAt,
          source: 'database',
        });
      }

      // Sort by createdAt descending (newest first)
      return chats.sort((a, b) => {
        const dateA = new Date(a.createdAt);
        const dateB = new Date(b.createdAt);
        return dateB - dateA;
      });
    } catch (error) {
      chatLogger.error('Error listing DB chats:', error);
      return [];
    }
  }

  /**
   * Elimina un chat completamente
   * @param {string} chatId - ID del chat a eliminar
   * @param {boolean} hardDelete - Si true, elimina permanentemente de la BD
   */
  static async deleteChat(chatId, hardDelete = false) {
    try {
      await ChatHistoryManager.deleteChat(chatId, hardDelete);
    } catch (error) {
      chatLogger.error('Error deleting chat from DB:', error);
    }
    removeSubAgent(chatId);
    chatLogger.info(`Chat deleted: ${chatId}`);
  }

  /**
   * Renombra un chat
   * @param {string} chatId - ID del chat
   * @param {string} name - Nuevo nombre
   */
  static async renameChat(chatId, name) {
    try {
      await ChatHistoryManager.updateChat(chatId, { name });
      return true;
    } catch (error) {
      chatLogger.error('Error renaming chat:', error);
      return false;
    }
  }

  /**
   * Fork a conversation up to (and including) a specific message timestamp.
   * @param {string} sourceChatId - Source chat ID
   * @param {string} upToTimestamp - ISO timestamp of the last message to include
   * @param {string} name - Optional name for the new chat
   * @returns {Promise<Object>} New chat with id, name, createdAt
   */
  static async forkChat(sourceChatId, upToTimestamp, name = null) {
    try {
      const chat = await ChatHistoryManager.forkChat(sourceChatId, upToTimestamp, name);
      chatLogger.info(`Chat forked: ${sourceChatId} → ${chat.id}`);
      return {
        id: chat.id,
        name: chat.name,
        createdAt: chat.createdAt,
      };
    } catch (error) {
      chatLogger.error('Error forking chat:', error);
      throw error;
    }
  }

  /**
   * Crea un nuevo chat y devuelve su ID
   * @param {string} name - Nombre opcional del chat
   * @param {Object} metadata - Metadata opcional inicial del chat
   * @returns {Promise<Object>} Chat creado con id, name, createdAt
   */
  static async createChat(name = null, metadata = {}) {
    try {
      const chat = await ChatHistoryManager.createChat(name, metadata);
      chatLogger.info(`Chat created: ${chat.id}`);
      return {
        id: chat.id,
        name: chat.name,
        createdAt: chat.createdAt,
      };
    } catch (error) {
      chatLogger.error('Error creating chat:', error);
      throw error;
    }
  }

  static async testMcpTool(toolName, toolArgs = {}) {
    if (!mcpClient || !mcpClient.isReady()) {
      throw new Error('MCP client not connected or not ready');
    }

    const availableTools = mcpClient.getTools().map((t) => t.name);
    if (!availableTools.includes(toolName)) {
      throw new Error(`Tool "${toolName}" not found. Available: ${availableTools.join(', ')}`);
    }

    chatLogger.info(`[testMcpTool] Executing tool: ${toolName}`);
    const result = await mcpClient.executeTool(toolName, toolArgs);
    chatLogger.info(`[testMcpTool] Tool "${toolName}" executed successfully`);
    return result;
  }
}
