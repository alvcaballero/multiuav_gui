/**
 * Pure derivations that turn a stored chat message into something the transcript can draw.
 * No React in here on purpose: the client has no test runner, so what can be pure is tested from
 * `server/test` (see test-chat-transcript.js).
 */

// --- Protocol conversion ---

export const convertMsg = (msg) => {
  if (!msg || !msg.message)
    return { role: 'assistant', type: 'error', content: 'Invalid message', status: 'error' };

  const status = msg.message.status || null;

  // Multipart content: array of input_text / input_image blocks (user messages with images)
  if (
    Array.isArray(msg.message.content) &&
    msg.message.content.some((b) => b.type === 'input_image')
  ) {
    return {
      role: msg.message.role,
      type: 'multipart',
      content: msg.message.content,
      status,
    };
  }

  // Orchestrator-authored nudge (e.g. retry after a malformed tool call),
  // not from the user or a subagent — never shown in the transcript.
  if (msg.message.type === 'system_directive') {
    return { role: 'system', type: 'system_directive', content: msg.message.content };
  }

  if (
    typeof msg.message.content === 'string' &&
    (msg.message.type === 'text' || !msg.message.type)
  ) {
    return {
      role: msg.message.role,
      content: msg.message.content,
      type: 'text',
      status,
    };
  }

  if (msg.message.type === 'reasoning') {
    return { role: 'assistant', type: 'reasoning', content: msg.message.content };
  }

  if (msg.message.type === 'message') {
    let contentText = '';
    if (typeof msg.message.content === 'string') {
      contentText = msg.message.content;
    } else if (Array.isArray(msg.message.content)) {
      const textPart = msg.message.content.find((c) => c.type === 'text');
      contentText = textPart ? textPart.text : msg.message.content[0]?.text || '';
    }
    return { role: msg.message.role, type: 'text', content: contentText };
  }

  if (msg.message.type === 'function_call' || msg.message.type === 'tool_call') {
    let args = msg.message.arguments || msg.message.content;
    if (typeof args === 'string') {
      try {
        args = JSON.parse(args);
      } catch {
        // Keep as string if not valid JSON
      }
    }

    return {
      role: 'assistant',
      type: 'function_call',
      content: args,
      name: msg.message.name,
      call_id: msg.message.call_id,
    };
  }

  // Async subagent result appended to the chat. Rendered as its own kind so it
  // never shows up as a user bubble — it arrives in the user turn, but the user
  // did not write it.
  if (msg.message.type === 'subagent_result') {
    let output = msg.message.output;

    if (typeof output === 'string') {
      try {
        output = JSON.parse(output);
      } catch {
        /* keep as string */
      }
    }

    return {
      role: 'assistant',
      type: 'subagent_result',
      content: output,
      name: msg.message.name,
      agentName: msg.message.agentName,
      subAgentChatId: msg.message.subAgentChatId,
      status: output?.status ?? status,
    };
  }

  if (msg.message.type === 'function_call_output') {
    let output = msg.message.output || msg.message.content;

    if (typeof output === 'string') {
      try {
        output = JSON.parse(output);
      } catch {
        /* keep as string */
      }
    }

    // MCP wraps the real payload inside content[0].text as a JSON string — unwrap it
    if (output?.content?.[0]?.text) {
      try {
        output = JSON.parse(output.content[0].text);
      } catch {
        /* keep outer */
      }
    }

    return {
      role: 'assistant',
      type: 'function_call_output',
      content: output,
      name: msg.message.name,
      call_id: msg.message.call_id,
    };
  }

  // Default fallback
  return {
    role: msg.message.role,
    type: 'text',
    content:
      typeof msg.message.content === 'string'
        ? msg.message.content
        : JSON.stringify(msg.message.content),
    status,
  };
};

export const formatMcpContent = (content) => {
  if (typeof content === 'string') {
    return content;
  }

  if (typeof content !== 'object' || content === null) {
    return JSON.stringify(content, null, 2);
  }

  if (content.content && Array.isArray(content.content)) {
    const textParts = content.content.flatMap((item) => {
      if (item.type !== 'text' || !item.text) return [];
      let text = item.text;
      if (typeof text === 'string') {
        text = text
          .replace(/\\n/g, '\n')
          .replace(/\\t/g, '\t')
          .replace(/\\"/g, '"')
          .replace(/\\\\/g, '\\');
      }
      return [text];
    });

    if (textParts.length > 0) {
      return textParts.join('\n');
    }
  }

  if (content.text && typeof content.text === 'string') {
    return content.text
      .replace(/\\n/g, '\n')
      .replace(/\\t/g, '\t')
      .replace(/\\"/g, '"')
      .replace(/\\\\/g, '\\');
  }

  return JSON.stringify(content, null, 2);
};

export const detectContentLanguage = (content) => {
  if (typeof content === 'string') {
    const trimmed = content.trim();
    if (
      (trimmed.startsWith('{') && trimmed.endsWith('}')) ||
      (trimmed.startsWith('[') && trimmed.endsWith(']'))
    ) {
      return 'json';
    }
    return 'text';
  }
  return 'json';
};

export const getResultSummary = (content) => {
  if (typeof content !== 'object' || content === null) {
    return { summary: 'Result', hasError: false };
  }

  const toolResult = content?.content?.[0]?.text;

  if (content.error) return { summary: 'Error', hasError: true };
  if (toolResult && toolResult.includes('MCP error')) return { summary: 'Result', hasError: true };
  if (content.success === false) return { summary: 'Fault', hasError: true };
  if (content.mission || content.missionId) return { summary: 'Created Mission', hasError: false };
  if (content.devices || Array.isArray(content))
    return { summary: 'Information getted successfully', hasError: false };
  return { summary: 'Result', hasError: false };
};

export const formatTimestamp = (ts) => {
  if (!ts) return null;
  const date = new Date(ts);
  if (isNaN(date.getTime())) return null;
  return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
};

/**
 * When a message was sent, for the line under or above it: just the time for today's, and the date
 * with it for any other day (and the year, once it is not this one) — a conversation can span days.
 */
export const formatMessageTime = (ts, now = new Date()) => {
  if (!ts) return null;
  const date = new Date(ts);
  if (isNaN(date.getTime())) return null;
  const time = date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  if (date.toDateString() === now.toDateString()) return time;
  const day = date.toLocaleDateString([], {
    month: 'short',
    day: 'numeric',
    ...(date.getFullYear() === now.getFullYear() ? {} : { year: 'numeric' }),
  });
  return `${day}, ${time}`;
};

// --- Derivation cache ---

/**
 * Memoises a pure unary derivation on the IDENTITY of its object argument.
 *
 * Every derivation here turns a message payload into something renderable, and all of them are
 * expensive for what they are: `convertMsg` can run three nested `JSON.parse` passes,
 * `formatMcpContent` runs four chained regex replacements over strings that reach ~145 KB in this
 * codebase's own transcripts, and both finish with a `JSON.stringify(..., null, 2)`. React calls
 * them again on every render, on payloads that never change — a message, once stored, is immutable.
 *
 * A WeakMap keyed on the payload object is the exact shape of that invariant: the derived value is
 * valid for as long as the object it came from is alive, and it is collected with it, so switching
 * chats or trimming a conversation to MAX_LIVE_MESSAGES cannot grow this cache without bound.
 * Keying on a message id would not work here — the server sends no stable id, and the client-side
 * fallback differs between a live message and the same message after a reload.
 *
 * Non-object arguments (a tool output that failed to parse stays a string) are passed straight
 * through: they cannot key a WeakMap, and for them these derivations are already O(1).
 */
export const memoizeByIdentity = (fn) => {
  const cache = new WeakMap();
  return (arg) => {
    if (arg === null || typeof arg !== 'object') return fn(arg);
    if (cache.has(arg)) return cache.get(arg);
    const value = fn(arg);
    cache.set(arg, value);
    return value;
  };
};

/**
 * Placeholder height for a text row the browser has not rendered yet (`contain-intrinsic-size`).
 *
 * A message's height varies wildly with its text — across this codebase's own transcripts a text
 * message runs 580 chars at the median, 4.1k at p75 and 39k at p99, so a single constant is wrong
 * by more than an order of magnitude for most of them, and a wrong estimate is exactly what makes a
 * scrollbar jump. Counting wrapped lines gets within the right ballpark, which is all that is
 * needed: once a row has been rendered the `auto` keyword makes the browser reuse its real height.
 */
const TEXT_CHARS_PER_LINE = 52; // ~400px of text column at 14px
const TEXT_LINE_HEIGHT = 21;
const TEXT_CHROME = 44; // label or meta line, padding, row gap
const TEXT_IMAGE_HEIGHT = 300; // matches the maxHeight on attached images
const MAX_TEXT_ESTIMATE = 6000;

const countWrappedLines = (text) =>
  text
    .split('\n')
    .reduce((total, line) => total + Math.max(1, Math.ceil(line.length / TEXT_CHARS_PER_LINE)), 0);

export const estimateTextHeight = (content) => {
  let lines = 0;
  let images = 0;

  if (typeof content === 'string') {
    lines = countWrappedLines(content);
  } else if (Array.isArray(content)) {
    for (const block of content) {
      if (block?.type === 'input_image') images += 1;
      else if (typeof block?.text === 'string') lines += countWrappedLines(block.text);
    }
  } else if (content != null) {
    lines = countWrappedLines(String(content));
  }

  const px = lines * TEXT_LINE_HEIGHT + images * TEXT_IMAGE_HEIGHT + TEXT_CHROME;
  // Bucketed so near-identical messages share one placeholder value.
  return Math.min(Math.ceil(px / 50) * 50, MAX_TEXT_ESTIMATE);
};

/**
 * Kinds that render as a collapsed row. Collapsed, all of them are the same height, so the row
 * carries one shared estimate and they need none of their own. Everything else — including
 * `convertMsg`'s fallbacks for an invalid or unrecognised payload — falls through to a text row,
 * and a text row under `content-visibility` MUST get a size or it collapses to nothing off screen.
 */
const COLLAPSED_TYPES = new Set([
  'system_directive',
  'reasoning',
  'function_call',
  'subagent_result',
  'function_call_output',
]);

/** Full view model of a raw message: protocol conversion, timestamp, size hint. */
export const getMessageView = memoizeByIdentity((msg) => {
  const converted = convertMsg(msg);
  return {
    ...converted,
    timestampLabel: formatMessageTime(msg?.timestamp),
    intrinsicSize: COLLAPSED_TYPES.has(converted.type)
      ? null
      : `auto ${estimateTextHeight(converted.content)}px`,
  };
});

/**
 * Display form of a tool/subagent payload. Language detection is folded in because it inspects the
 * FORMATTED string — computing it here avoids a `.trim()` copy of a six-figure-byte string on every
 * render.
 */
export const getFormattedResult = memoizeByIdentity((content) => {
  const text = formatMcpContent(content);
  return { text, language: detectContentLanguage(text) };
});

export const getCachedResultSummary = memoizeByIdentity(getResultSummary);

export const getPrettyJson = memoizeByIdentity((content) =>
  typeof content === 'object' ? JSON.stringify(content, null, 2) : content,
);

// --- Tool calls: one row per call, with its result ---

/**
 * The transcript stores a tool call and its result as two messages; the operator reads them as one
 * action. This folds each `function_call_output` into the `function_call` with the same `call_id`.
 *
 * A result whose call is not in the list (the call fell off the 300 live messages, or sits on the
 * other side of a pagination boundary) stays as a row of its own, with no input. A call with no
 * result yet is still running, or never answered — the caller decides which.
 *
 * Messages the operator never sees (`system_directive`) are dropped here.
 *
 * @param {Array<object>} messages - raw stored messages, oldest first
 * @returns {Array<{kind: 'message', key: string, message: object} | {kind: 'tool', key: string, call: object|null, output: object|null}>}
 */
export const groupTranscript = (messages) => {
  const items = [];
  const openCalls = new Map(); // call_id -> the tool item still waiting for its result

  for (const message of messages) {
    const view = getMessageView(message);
    const key = message.id || message.timestamp;

    if (view.type === 'system_directive') continue;

    if (view.type === 'function_call') {
      const item = { kind: 'tool', key, call: message, output: null };
      items.push(item);
      if (view.call_id) openCalls.set(view.call_id, item);
      continue;
    }

    if (view.type === 'function_call_output') {
      const open = view.call_id ? openCalls.get(view.call_id) : null;
      if (open && !open.output) {
        open.output = message;
        openCalls.delete(view.call_id);
      } else {
        items.push({ kind: 'tool', key, call: null, output: message });
      }
      continue;
    }

    items.push({ kind: 'message', key, message });
  }

  return items;
};

const itemTime = (item) => {
  const timestamp =
    item.kind === 'tool' ? (item.call ?? item.output)?.timestamp : item.message?.timestamp;
  const time = new Date(timestamp).getTime();
  return Number.isFinite(time) ? time : null;
};

/**
 * Puts the decided approvals into the transcript where they happened: before the first item that is
 * newer than the decision (the tool call an approval lets through comes right after it). An item
 * with no usable time is skipped when looking for that place; a decision newer than everything goes
 * last.
 *
 * @param {Array<object>} items - the result of `groupTranscript`
 * @param {Array<{requestId: string, at: string}>} resolved
 * @returns {Array<object>} `items` with `{kind: 'resolved', key, resolved}` rows added
 */
export const mergeResolved = (items, resolved) => {
  if (!resolved?.length) return items;

  const merged = [...items];
  for (const entry of [...resolved].sort((a, b) => new Date(a.at) - new Date(b.at))) {
    const at = new Date(entry.at).getTime();
    const index = merged.findIndex((item) => {
      if (item.kind === 'resolved') return false;
      const time = itemTime(item);
      return time !== null && time > at;
    });
    const row = { kind: 'resolved', key: `resolved-${entry.requestId}`, resolved: entry };
    if (index === -1) merged.push(row);
    else merged.splice(index, 0, row);
  }
  return merged;
};

/**
 * What goes beside a tool's name in its collapsed row: the first couple of plain argument values,
 * enough to tell two calls of the same tool apart ("uav_1", "lawnmower · 6 WP"). Nested values are
 * left for the expanded view.
 */
export const summarizeArgs = (args, maxChars = 48) => {
  if (args == null) return '';
  let text;
  if (typeof args === 'string') {
    text = args;
  } else if (typeof args === 'object') {
    const plain = Object.values(args).filter(
      (value) => ['string', 'number', 'boolean'].includes(typeof value) && String(value) !== '',
    );
    text = plain.slice(0, 2).join(' · ');
  } else {
    text = String(args);
  }
  text = text.replace(/\s+/g, ' ').trim();
  return text.length > maxChars ? `${text.slice(0, maxChars - 1)}…` : text;
};

/** "0.4s", "12s", "1m 05s" between the call and its result; null when either time is unusable. */
export const durationLabel = (from, to) => {
  // `new Date(null)` is the epoch, not an invalid date: a missing time must not read as 1970.
  if (!from || !to) return null;
  const start = new Date(from).getTime();
  const end = new Date(to).getTime();
  if (!Number.isFinite(start) || !Number.isFinite(end) || end < start) return null;
  const seconds = (end - start) / 1000;
  if (seconds < 10) return `${seconds.toFixed(1)}s`;
  if (seconds < 60) return `${Math.round(seconds)}s`;
  return `${Math.floor(seconds / 60)}m ${String(Math.round(seconds % 60)).padStart(2, '0')}s`;
};

/**
 * The rows of a command's parameters for the approval card. Every top-level key is shown with its
 * whole value — an object or array as compact JSON, never cut — because the operator approves the
 * command as it will execute, not a summary of it.
 */
export const paramRows = (input) => {
  if (input == null) return [];
  if (typeof input !== 'object' || Array.isArray(input)) {
    return [{ key: 'input', value: JSON.stringify(input) }];
  }
  return Object.entries(input).map(([key, value]) => ({
    key,
    value: typeof value === 'string' ? value : JSON.stringify(value),
  }));
};
