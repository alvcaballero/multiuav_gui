import { EveAgents, EveDefaultAgent } from '../../config/config.js';
import { chatLogger } from '../../common/logger.js';

/**
 * Thin HTTP client for an eve agent (`gcs_eevee_assistant`), over its `<base>/v1/session` API.
 * Kept separate from `mcpClient.js`: that one is this process acting as an MCP tool CLIENT for the
 * legacy orchestrator; eve is a whole different orchestrator this process talks to as an HTTP peer.
 *
 * A "base" is the agent's mount point up to, not including, `/v1` — `http://host:2000/eve` for
 * `eve dev`, `http://host:3300/eve/assistant` for `dev:all`. eve cannot list its agents, so the
 * registry is `config.EveAgents`. Never build a URL from an event's `streamPath`: eve fills it with
 * its agent-local route (`/eve/v1/...`), which is a 404 under `dev:all`.
 */

/** A non-2xx answer from eve. `code` is eve's own error code when the body carries one. */
export class EveHttpError extends Error {
  constructor(message, { status, code = null, body = '' } = {}) {
    super(message);
    this.name = 'EveHttpError';
    this.status = status;
    this.code = code;
    this.body = body;
  }
}

/** The session's history is gone: eve's World deleted it after its retention period (410). */
export class EveSessionExpiredError extends EveHttpError {
  constructor(details) {
    super('eve session history is no longer available (retention expired)', details);
    this.name = 'EveSessionExpiredError';
  }
}

/**
 * What to tell the operator when a call to eve fails. A network error reads "fetch failed", which
 * says nothing; what they can act on is that nobody is listening (eve is not running) or that the
 * agent refuses the GCS. `base` is only used for the host: the URL itself can carry credentials.
 * @param {Error} error
 * @param {string} agent
 * @param {string} [base] - the agent's configured base URL
 */
export const explainEveError = (error, agent, base) => {
  let where = '';
  try {
    if (base) where = ` at ${new URL(base).host}`;
  } catch {
    // a malformed base: say nothing about where rather than fail while explaining a failure
  }

  if (error instanceof EveHttpError) {
    if (error.status === 401 || error.status === 403) {
      return `The eve agent "${agent}"${where} rejected the GCS credentials (${error.status}).`;
    }
    return error.message;
  }
  if (
    /fetch failed|ECONNREFUSED|ENOTFOUND|EHOSTUNREACH|timeout|aborted/i.test(`${error?.message} ${error?.cause?.code}`)
  ) {
    return `The eve agent "${agent}"${where} is not reachable. Is it running?`;
  }
  return error?.message || 'Error talking to the eve agent';
};

const parseErrorCode = (body) => {
  try {
    const parsed = JSON.parse(body);
    return parsed?.code ?? parsed?.error?.code ?? null;
  } catch {
    return null;
  }
};

// `session_not_ready` means the session exists but is not accepting input yet; eve's own TS client
// retries it for ~20s. Same budget here.
const NOT_READY_BACKOFF_MS = [250, 500, 1000, 2000, 4000, 8000];

const defaultSleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

/**
 * @param {object} [deps] - injected for tests
 * @param {typeof fetch} [deps.fetch]
 * @param {(ms: number) => Promise<void>} [deps.sleep]
 * @param {Record<string, string>} [deps.agents] - name → base URL
 * @param {string} [deps.defaultAgent]
 */
export function createEveClient({
  fetch: fetchImpl = globalThis.fetch,
  sleep = defaultSleep,
  agents = EveAgents,
  defaultAgent = EveDefaultAgent,
} = {}) {
  const baseOf = (agent) => {
    const base = agents[agent ?? defaultAgent];
    if (!base) throw new Error(`Unknown eve agent '${agent}'. Configured: ${Object.keys(agents).join(', ') || 'none'}`);
    return base;
  };

  const urlOf = (agent, path) => `${baseOf(agent)}/v1${path}`;

  const toError = async (what, res) => {
    const body = await res.text();
    const details = { status: res.status, code: parseErrorCode(body), body };
    if (res.status === 410) return new EveSessionExpiredError(details);
    return new EveHttpError(`eve ${what} failed: ${res.status} ${body}`, details);
  };

  // POST/GET that retries `session_not_ready` and nothing else: any other 4xx/5xx is the caller's.
  const request = async (agent, path, what, { method = 'GET', body } = {}) => {
    for (let attempt = 0; ; attempt++) {
      const res = await fetchImpl(urlOf(agent, path), {
        method,
        headers: body === undefined ? undefined : { 'Content-Type': 'application/json' },
        body: body === undefined ? undefined : JSON.stringify(body),
      });
      if (res.ok) return res.status === 204 ? null : res.json();

      const error = await toError(what, res);
      if (error.code === 'session_not_ready' && attempt < NOT_READY_BACKOFF_MS.length) {
        await sleep(NOT_READY_BACKOFF_MS[attempt]);
        continue;
      }
      throw error;
    }
  };

  return {
    baseOf,
    urlOf,

    /** Creates a session and sends its first message in one call. */
    async createSession(agent, message) {
      const body = await request(agent, '/session', 'session create', { method: 'POST', body: { message } });
      return { sessionId: body.sessionId, status: body.status };
    },

    /** A follow-up message. While a tool approval is open, free text CANCELS it (eve outcome `ignored`). */
    sendMessage: (agent, sessionId, message) =>
      request(agent, `/session/${sessionId}`, 'send message', { method: 'POST', body: { message } }),

    /**
     * Answers open input requests. eve accepts `message` OR `inputResponses`, never both.
     * @param {Array<{requestId: string, optionId: string, text?: string}>} responses
     */
    sendInputResponses: (agent, sessionId, responses) =>
      request(agent, `/session/${sessionId}`, 'input response', {
        method: 'POST',
        body: { inputResponses: responses },
      }),

    /** Asynchronous: the answer is 202 and the end shows up in the stream as `turn.cancelled`. */
    cancelSession: (agent, sessionId, turnId) =>
      request(agent, `/session/${sessionId}/cancel`, 'cancel', {
        method: 'POST',
        body: turnId ? { turnId } : {},
      }),

    getHealth: (agent) => request(agent, '/health', 'health'),
    getInfo: (agent) => request(agent, '/info', 'info'),

    /**
     * Streams a session's events as NDJSON and yields them parsed.
     *
     * The stream is durable: without `startIndex` it replays from event 0 on every GET. Callers that
     * keep a cursor MUST pass the absolute count of events already consumed — see `EveRelay`.
     * `session_not_ready` is retried like the POSTs; a `410` becomes `EveSessionExpiredError`.
     *
     * @param {string} agent
     * @param {string} sessionId
     * @param {{startIndex?: number, signal?: AbortSignal}} [options]
     * @returns {AsyncGenerator<object>}
     */
    async *streamEvents(agent, sessionId, { startIndex, signal } = {}) {
      const url = new URL(urlOf(agent, `/session/${sessionId}/stream`));
      if (typeof startIndex === 'number') url.searchParams.set('startIndex', String(startIndex));

      let res;
      for (let attempt = 0; ; attempt++) {
        res = await fetchImpl(url, { signal });
        if (res.ok) break;
        const error = await toError('stream open', res);
        if (error.code === 'session_not_ready' && attempt < NOT_READY_BACKOFF_MS.length) {
          await sleep(NOT_READY_BACKOFF_MS[attempt]);
          continue;
        }
        throw error;
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
        // A consumer that breaks out of the loop leaves the HTTP stream open; cancel it, not just unlock.
        await reader.cancel().catch(() => {});
        reader.releaseLock();
      }
    },
  };
}

export const eveClient = createEveClient();
