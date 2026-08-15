/**
 * Projection of an LLM-protocol message payload onto the flat, queryable columns
 * of the ChatMessage table.
 *
 * ── Why this exists ────────────────────────────────────────────────────────────
 * `messageData` is the SINGLE SOURCE OF TRUTH: the raw provider payload, and the
 * only thing replayed back to the LLM (see ChatHistoryManager.loadHistory).
 * `role` / `type` / `content` are DERIVED from it and exist purely so the rows can
 * be filtered in SQL and read by a human, which a JSON column cannot do reliably
 * across dialects (SQLite JSON LIKE is sensitive to key order and spacing).
 *
 * Keeping the derivation in one pure function means the invariant
 * "derived columns are a function of messageData, nothing else" is enforceable
 * and testable, instead of living inside a branch of a persistence method.
 */
import { chatLogger } from '../../common/logger.js';

/** Max length persisted in the `content` column for tool outputs. */
export const CONTENT_MAX_LENGTH = 500;

/** Values accepted by the `role` column. Mirrors the LLM protocol vocabulary. */
export const MESSAGE_ROLES = Object.freeze(['user', 'assistant', 'system', 'tool', 'unknown']);

/** Values accepted by the `type` column. Mirrors the projected payload kinds. */
export const MESSAGE_TYPES = Object.freeze([
  'text',
  'tool_call',
  'tool_result',
  'subagent_result',
  'reasoning',
  'multipart',
  'unknown',
]);

/**
 * Flattens an array of content blocks into plain text.
 * Handles both OpenAI Responses (`output_text`) and Chat Completions (`text`) shapes.
 * @param {Array} blocks
 * @returns {string|null}
 */
function joinTextBlocks(blocks) {
  const text = blocks
    .filter((b) => b && (b.type === 'output_text' || b.type === 'text'))
    .map((b) => b.text)
    .filter((t) => typeof t === 'string')
    .join('\n');

  return text.length > 0 ? text : null;
}

/**
 * Projects a raw provider message payload onto the derived ChatMessage columns.
 *
 * Pure: no I/O, no DB access. Unknown payload kinds are logged (never swallowed)
 * and fall back to a best-effort projection so an unrecognised provider shape can
 * never silently produce an empty row.
 *
 * @param {object} message - Raw payload, stored verbatim in `messageData`
 * @returns {{role: string, type: string, content: string|null}} Derived columns
 */
export function projectMessage(message) {
  if (!message || typeof message !== 'object') {
    chatLogger.warn(`[projectMessage] Non-object payload (${typeof message}); projecting as unknown`);
    return { role: 'unknown', type: 'unknown', content: null };
  }

  switch (message.type) {
    case 'function_call':
    case 'tool_call':
      return { role: 'assistant', type: 'tool_call', content: `Tool call: ${message.name}` };

    case 'function_call_output':
      return {
        role: 'tool',
        type: 'tool_result',
        content: typeof message.output === 'string' ? message.output.slice(0, CONTENT_MAX_LENGTH) : null,
      };

    // Async result of a subagent, appended to the parent chat as a NEW message
    // (never as a replayed function_call_output — the parent's tool pair closed
    // long before the subagent answered). `role: 'user'` because that is the only
    // protocol role every provider replays verbatim; `from` carries the real
    // sender. `output` is a JSON string, same convention as function_call_output.
    case 'subagent_result':
      return {
        role: 'user',
        type: 'subagent_result',
        content: typeof message.output === 'string' ? message.output.slice(0, CONTENT_MAX_LENGTH) : null,
      };

    case 'reasoning':
      return {
        role: 'assistant',
        type: 'reasoning',
        content: Array.isArray(message.summary) ? message.summary.map((s) => s?.text || s).join('\n') : null,
      };

    // Bare text part, as emitted by Gemini: {type:'text', text:'...'} with no role.
    case 'text':
      return {
        role: normalizeRole(message.role) === 'unknown' ? 'assistant' : normalizeRole(message.role),
        type: 'text',
        content: typeof message.text === 'string' ? message.text : null,
      };

    case 'message': {
      const role = normalizeRole(message.role) === 'unknown' ? 'assistant' : normalizeRole(message.role);
      if (Array.isArray(message.content)) {
        return { role, type: 'text', content: joinTextBlocks(message.content) };
      }
      return {
        role,
        type: 'text',
        content: typeof message.content === 'string' ? message.content : null,
      };
    }

    default:
      break;
  }

  // No explicit `type`: a plain {role, content} message (the shape used by user
  // input and by handlers that do not speak the Responses API).
  if (message.type === undefined) {
    if (Array.isArray(message.content)) {
      // Multimodal input (input_text + input_image blocks). Text is projected for
      // search/debug; the images stay in messageData.
      return { role: normalizeRole(message.role), type: 'multipart', content: joinTextBlocks(message.content) };
    }
    return {
      role: normalizeRole(message.role),
      type: 'text',
      content: typeof message.content === 'string' ? message.content : null,
    };
  }

  // An explicit but unrecognised type: a provider shape this projection does not
  // know yet. Surface it loudly — a silent null here is how rows go blank.
  chatLogger.warn(`[projectMessage] Unhandled message type "${message.type}"; storing as unknown`);
  return {
    role: normalizeRole(message.role),
    type: 'unknown',
    content: typeof message.content === 'string' ? message.content : null,
  };
}

/**
 * Coerces a role to a value the `role` ENUM accepts.
 * @param {*} role
 * @returns {string} A member of MESSAGE_ROLES
 */
export function normalizeRole(role) {
  return MESSAGE_ROLES.includes(role) ? role : 'unknown';
}

/**
 * Coerces a type to a value the `type` ENUM accepts.
 * @param {*} type
 * @returns {string} A member of MESSAGE_TYPES
 */
export function normalizeType(type) {
  return MESSAGE_TYPES.includes(type) ? type : 'unknown';
}
