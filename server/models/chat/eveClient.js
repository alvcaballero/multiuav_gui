import { EveUrl } from '../../config/config.js';
import { chatLogger } from '../../common/logger.js';

/**
 * Thin HTTP client for the eve agent (`gcs_eevee_assistant`), spoken over its
 * `/eve/v1/session` API. Kept separate from `mcpClient.js`: that one is this
 * process acting as an MCP tool CLIENT for the legacy orchestrator; eve is a
 * whole different orchestrator that this process talks to as an HTTP peer.
 */

/**
 * Creates a new eve session and sends the first message in one call.
 * @param {string} message
 * @returns {Promise<{sessionId: string, status: string}>}
 */
export async function createEveSession(message) {
  const res = await fetch(`${EveUrl}/eve/v1/session`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ message }),
  });
  if (!res.ok) {
    throw new Error(`eve session create failed: ${res.status} ${await res.text()}`);
  }
  const body = await res.json();
  return { sessionId: body.sessionId, status: body.status };
}

/**
 * Sends a follow-up message into an existing eve session.
 * @param {string} sessionId
 * @param {string} message
 */
export async function sendEveMessage(sessionId, message) {
  const res = await fetch(`${EveUrl}/eve/v1/session/${sessionId}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ message }),
  });
  if (!res.ok) {
    throw new Error(`eve send message failed: ${res.status} ${await res.text()}`);
  }
  return res.json();
}

/**
 * Streams an eve session's events as NDJSON (`application/x-ndjson`, one
 * `MessageStreamEvent` per line) and yields them parsed, one at a time.
 *
 * The stream is durable: without `startIndex` it replays from event 0 on
 * every GET, not just new events since the last read. Callers that persist a
 * cursor MUST pass the absolute event count already consumed — see
 * `EveOrchestrator` for how that cursor is tracked per chat.
 *
 * @param {string} sessionId
 * @param {{startIndex?: number}} [options]
 * @returns {AsyncGenerator<object>}
 */
export async function* streamEveEvents(sessionId, { startIndex } = {}) {
  const url = new URL(`${EveUrl}/eve/v1/session/${sessionId}/stream`);
  if (typeof startIndex === 'number') url.searchParams.set('startIndex', String(startIndex));

  const res = await fetch(url);
  if (!res.ok) {
    throw new Error(`eve stream open failed: ${res.status} ${await res.text()}`);
  }

  const reader = res.body.getReader();
  const decoder = new TextDecoder();
  let buffer = '';

  try {
    while (true) {
      const { done, value } = await reader.read();
      if (done) break;
      buffer += decoder.decode(value, { stream: true });

      let newlineIndex;
      while ((newlineIndex = buffer.indexOf('\n')) >= 0) {
        const line = buffer.slice(0, newlineIndex).trim();
        buffer = buffer.slice(newlineIndex + 1);
        if (!line) continue;
        try {
          yield JSON.parse(line);
        } catch (error) {
          chatLogger.error('eveClient: failed to parse NDJSON line', { line, error: error.message });
        }
      }
    }
  } finally {
    reader.releaseLock();
  }
}
