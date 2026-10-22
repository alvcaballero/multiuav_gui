import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { createEveClient, EveHttpError, EveSessionExpiredError, explainEveError } from '../models/chat/eveClient.js';
import { normalizeEveBase } from '../config/config.js';

const AGENTS = { assistant: 'http://gcs:3300/eve/assistant', solo: 'http://gcs:2000/eve' };

const json = (body, status = 200) =>
  new Response(JSON.stringify(body), { status, headers: { 'Content-Type': 'application/json' } });

/** A fetch that answers from a queue (a function answers a fresh response each time) and remembers what it was asked. */
const makeFetch = (...responses) => {
  const calls = [];
  const fetch = async (url, init = {}) => {
    calls.push({ url: String(url), method: init.method ?? 'GET', body: init.body ? JSON.parse(init.body) : undefined });
    const next = responses.length > 1 ? responses.shift() : responses[0];
    return typeof next === 'function' ? next() : next.clone();
  };
  return { fetch, calls };
};

const ndjson = (chunks) =>
  new Response(
    new ReadableStream({
      start(controller) {
        for (const chunk of chunks) controller.enqueue(new TextEncoder().encode(chunk));
        controller.close();
      },
    }),
    { status: 200, headers: { 'Content-Type': 'application/x-ndjson' } }
  );

describe('eve base URL', () => {
  it('keeps a base that already names the mount point', () => {
    assert.equal(normalizeEveBase('http://h:3300/eve/assistant'), 'http://h:3300/eve/assistant');
    assert.equal(normalizeEveBase('http://h:2000/eve/'), 'http://h:2000/eve');
  });

  it('adds /eve to the old EVE_URL shape (host only), which used to get /eve/v1 appended by the client', () => {
    assert.equal(normalizeEveBase('http://127.0.0.1:2000'), 'http://127.0.0.1:2000/eve');
    assert.equal(normalizeEveBase('http://127.0.0.1:2000/'), 'http://127.0.0.1:2000/eve');
  });

  it('does not mistake a host called eve for the mount point', () => {
    assert.equal(normalizeEveBase('http://eve:2000'), 'http://eve:2000/eve');
  });
});

describe('eveClient — requests', () => {
  it('builds every URL from the agent base plus /v1, for both dev modes', async () => {
    const { fetch, calls } = makeFetch(() => json({ sessionId: 'wrun_1', status: 'accepted' }, 202));
    const client = createEveClient({ fetch, agents: AGENTS, defaultAgent: 'assistant' });

    await client.createSession('assistant', 'hola');
    await client.createSession('solo', 'hola');

    assert.equal(calls[0].url, 'http://gcs:3300/eve/assistant/v1/session');
    assert.equal(calls[1].url, 'http://gcs:2000/eve/v1/session');
    assert.deepEqual(calls[0].body, { message: 'hola' });
  });

  it('refuses an agent that is not configured instead of guessing a URL', async () => {
    const client = createEveClient({ fetch: makeFetch(json({})).fetch, agents: AGENTS, defaultAgent: 'assistant' });
    await assert.rejects(() => client.sendMessage('ghost', 's', 'x'), /Unknown eve agent 'ghost'/);
  });

  it('answers input requests with inputResponses and NOT with a message (eve accepts one or the other)', async () => {
    const { fetch, calls } = makeFetch(() => json({ ok: true }, 202));
    const client = createEveClient({ fetch, agents: AGENTS });

    await client.sendInputResponses('assistant', 'wrun_1', [{ requestId: 'r1', optionId: 'cancel', text: 'no' }]);

    assert.equal(calls[0].url, 'http://gcs:3300/eve/assistant/v1/session/wrun_1');
    assert.deepEqual(calls[0].body, { inputResponses: [{ requestId: 'r1', optionId: 'cancel', text: 'no' }] });
    assert.equal('message' in calls[0].body, false);
  });

  it('cancels with an optional turnId', async () => {
    const { fetch, calls } = makeFetch(() => json({ ok: true, status: 'accepted' }, 202));
    const client = createEveClient({ fetch, agents: AGENTS });

    await client.cancelSession('assistant', 'wrun_1');
    await client.cancelSession('assistant', 'wrun_1', 'turn_3');

    assert.equal(calls[0].url, 'http://gcs:3300/eve/assistant/v1/session/wrun_1/cancel');
    assert.deepEqual(calls[0].body, {});
    assert.deepEqual(calls[1].body, { turnId: 'turn_3' });
  });

  it('retries session_not_ready with backoff and then succeeds', async () => {
    const notReady = () => json({ code: 'session_not_ready' }, 409);
    const { fetch, calls } = makeFetch(notReady, notReady, () => json({ ok: true }, 202));
    const sleeps = [];
    const client = createEveClient({ fetch, agents: AGENTS, sleep: async (ms) => sleeps.push(ms) });

    await client.sendMessage('assistant', 'wrun_1', 'hola');

    assert.equal(calls.length, 3);
    assert.deepEqual(sleeps, [250, 500]);
  });

  it('does not retry other conflicts: session_stranded is for the caller to deal with', async () => {
    const { fetch, calls } = makeFetch(() => json({ code: 'session_stranded' }, 409));
    const client = createEveClient({ fetch, agents: AGENTS, sleep: async () => {} });

    await assert.rejects(
      () => client.sendMessage('assistant', 'wrun_1', 'hola'),
      (error) => error instanceof EveHttpError && error.status === 409 && error.code === 'session_stranded'
    );
    assert.equal(calls.length, 1);
  });
});

describe('eveClient — the stream', () => {
  const collect = async (generator) => {
    const events = [];
    for await (const event of generator) events.push(event);
    return events;
  };

  it('asks for the stream from the cursor', async () => {
    const { fetch, calls } = makeFetch(() => ndjson(['']));
    const client = createEveClient({ fetch, agents: AGENTS });

    await collect(client.streamEvents('assistant', 'wrun_1', { startIndex: 17 }));

    assert.equal(calls[0].url, 'http://gcs:3300/eve/assistant/v1/session/wrun_1/stream?startIndex=17');
  });

  it('parses events split across chunks and skips a line that is not JSON', async () => {
    const { fetch } = makeFetch(() => ndjson(['{"type":"a"}\n{"ty', 'pe":"b"}\nnot json\n', '{"type":"c', '"}\n']));
    const client = createEveClient({ fetch, agents: AGENTS });

    const events = await collect(client.streamEvents('assistant', 'wrun_1', { startIndex: 0 }));

    assert.deepEqual(
      events.map((event) => event.type),
      ['a', 'b', 'c']
    );
  });

  it('turns a 410 into "the history is gone" so the relay can stop retrying', async () => {
    const { fetch } = makeFetch(() => json({ code: 'session_expired' }, 410));
    const client = createEveClient({ fetch, agents: AGENTS });

    await assert.rejects(
      () => collect(client.streamEvents('assistant', 'wrun_1', { startIndex: 0 })),
      (error) => error instanceof EveSessionExpiredError && error.status === 410
    );
  });
});

describe('explainEveError — what the operator reads when eve cannot be reached', () => {
  const base = 'http://user:secret@gcs:3300/eve/assistant';

  it('says eve is not running, with the host, instead of "fetch failed"', () => {
    const error = new TypeError('fetch failed', { cause: { code: 'ECONNREFUSED' } });

    const text = explainEveError(error, 'assistant', base);

    assert.equal(text, 'The eve agent "assistant" at gcs:3300 is not reachable. Is it running?');
    assert.ok(!text.includes('secret'), 'only the host, never the credentials in the URL');
  });

  it('says the agent refuses the GCS when it answers 401 or 403', () => {
    const error = new EveHttpError('eve send message failed: 401', { status: 401 });

    assert.match(explainEveError(error, 'assistant', base), /rejected the GCS credentials \(401\)/);
  });

  it('keeps the message of any other answer from eve, and survives a missing or broken base', () => {
    assert.equal(
      explainEveError(new EveHttpError('eve stream open failed: 500 boom', { status: 500 }), 'a', base),
      'eve stream open failed: 500 boom'
    );
    assert.equal(explainEveError(new Error('boom'), 'a', 'not a url'), 'boom');
    assert.equal(
      explainEveError(new TypeError('fetch failed'), 'a'),
      'The eve agent "a" is not reachable. Is it running?'
    );
  });
});
