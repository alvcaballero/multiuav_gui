import { createHash, randomUUID } from 'node:crypto';
import { TOOL_APPROVAL_REQUIRED, TOOL_APPROVAL_TTL_MS } from '../../config/config.js';

/**
 * A tool call was approved with one input and reached execution with another.
 * Never recoverable: the operator authorised a different action.
 */
export class ToolApprovalMismatchError extends Error {
  constructor({ callId, toolName, approvedHash, actualHash }) {
    super(
      `Approved input for "${toolName}" (call ${callId}) does not match the input about to run: ` +
        `approved ${approvedHash}, got ${actualHash}`
    );
    this.name = 'ToolApprovalMismatchError';
    this.code = 'tool_approval_input_mismatch';
    this.callId = callId;
    this.toolName = toolName;
    this.approvedHash = approvedHash;
    this.actualHash = actualHash;
  }
}

/**
 * A tool that moves an aircraft was reached with no human approval on record.
 * Distinct from a failure: nothing ran.
 */
export class ToolApprovalRequiredError extends Error {
  constructor({ callId, toolName }) {
    super(`"${toolName}" requires human approval before it can run (call ${callId})`);
    this.name = 'ToolApprovalRequiredError';
    this.code = 'tool_approval_required';
    this.callId = callId;
    this.toolName = toolName;
  }
}

export class ToolApprovalExpiredError extends Error {
  constructor({ callId, toolName, expiresAt }) {
    super(`Approval for "${toolName}" (call ${callId}) expired at ${expiresAt}`);
    this.name = 'ToolApprovalExpiredError';
    this.code = 'tool_approval_expired';
    this.callId = callId;
    this.toolName = toolName;
    this.expiresAt = expiresAt;
  }
}

/**
 * Deterministic serialisation, so the same logical input always produces the
 * same bytes and therefore the same hash.
 *
 * Object keys are sorted; array order is NEVER touched — waypoint order is the
 * mission. Non-finite numbers throw instead of following `JSON.stringify` into
 * `null`: an altitude that silently becomes null is a flight hazard, not a
 * serialisation detail.
 */
export const canonicalizeInput = (value) => {
  if (value === null) return 'null';

  const type = typeof value;

  if (type === 'number') {
    if (!Number.isFinite(value)) {
      throw new TypeError(`canonicalizeInput: non-finite number (${value}) cannot be canonicalised`);
    }
    return JSON.stringify(value);
  }

  if (type === 'string' || type === 'boolean') return JSON.stringify(value);

  if (Array.isArray(value)) {
    // `undefined` in an array is a hole; JSON.stringify writes `null` there and
    // so does the payload the tool receives, so match it rather than diverge.
    return `[${value.map((item) => (item === undefined ? 'null' : canonicalizeInput(item))).join(',')}]`;
  }

  if (type === 'object') {
    const entries = Object.keys(value)
      .filter((key) => value[key] !== undefined)
      .sort()
      .map((key) => `${JSON.stringify(key)}:${canonicalizeInput(value[key])}`);
    return `{${entries.join(',')}}`;
  }

  throw new TypeError(`canonicalizeInput: cannot canonicalise ${type}`);
};

export const computeInputHash = (input) => `sha256:${createHash('sha256').update(canonicalizeInput(input)).digest('hex')}`;

export const requiresApproval = (toolName) => TOOL_APPROVAL_REQUIRED.includes(toolName);

/**
 * Freezes the exact input that will be executed.
 *
 * `input` must be the RESOLVED input — context params already merged in, as
 * `executeToolCalls` does — not the raw arguments the model emitted. Freezing
 * the raw arguments would leave the merged-in context free to change between
 * approval and execution without the hash noticing.
 */
export const freezeToolCall = ({ callId, toolName, input, ttlMs = TOOL_APPROVAL_TTL_MS }) => {
  const frozenAt = new Date();
  return {
    requestId: randomUUID(),
    callId,
    toolName,
    input,
    inputHash: computeInputHash(input),
    frozenAt: frozenAt.toISOString(),
    expiresAt: new Date(frozenAt.getTime() + ttlMs).toISOString(),
  };
};

export const isExpired = (frozen, now = Date.now()) => new Date(frozen.expiresAt).getTime() <= now;

/**
 * The gate. Call immediately before handing the input to the tool.
 *
 * Verifies the input about to run is byte-identical to the one the operator saw,
 * and that the approval has not aged out. It deliberately does NOT re-derive
 * anything: `frozen.input` is the value to execute, and this only proves nobody
 * swapped it.
 */
export const assertApprovedInputMatches = (frozen, inputToExecute, now = Date.now()) => {
  if (isExpired(frozen, now)) {
    throw new ToolApprovalExpiredError(frozen);
  }

  const actualHash = computeInputHash(inputToExecute);
  if (actualHash !== frozen.inputHash) {
    throw new ToolApprovalMismatchError({
      callId: frozen.callId,
      toolName: frozen.toolName,
      approvedHash: frozen.inputHash,
      actualHash,
    });
  }

  return true;
};
