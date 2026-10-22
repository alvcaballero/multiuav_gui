import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import {
  groupTranscript,
  mergeResolved,
  summarizeArgs,
  durationLabel,
  formatMessageTime,
  paramRows,
} from '../../client/src/common/util/chatMessage.js';
import { groupHistory, dayGroupOf, historyTimeLabel } from '../../client/src/common/util/chatHistory.js';

let counter = 0;
const stored = (message, timestamp = '2026-10-22T10:41:00.000Z') => ({
  message,
  timestamp,
  id: `m${++counter}`,
});
const call = (call_id, name = 'get_telemetry', args = { device: 'uav_1' }) =>
  stored({ type: 'function_call', name, call_id, arguments: JSON.stringify(args) });
const output = (call_id, name = 'get_telemetry', out = { ok: true }) =>
  stored({ type: 'function_call_output', name, call_id, output: JSON.stringify(out) });
const text = (role, content) => stored({ role, content, type: 'text' });

describe('groupTranscript: a tool call and its result are one row', () => {
  it('folds each result into the call with the same call_id', () => {
    const c = call('a');
    const o = output('a');
    const items = groupTranscript([text('user', 'hi'), c, o, text('assistant', 'done')]);

    assert.deepEqual(
      items.map((i) => i.kind),
      ['message', 'tool', 'message'],
    );
    assert.equal(items[1].call, c);
    assert.equal(items[1].output, o);
  });

  it('keeps a call with no result yet (still running) with output null', () => {
    const items = groupTranscript([call('a')]);
    assert.equal(items.length, 1);
    assert.equal(items[0].output, null);
  });

  it('pairs interleaved calls by id, not by position', () => {
    const [a, b] = [call('a', 'one'), call('b', 'two')];
    const [outB, outA] = [output('b', 'two'), output('a', 'one')];
    const items = groupTranscript([a, b, outB, outA]);

    assert.equal(items.length, 2);
    assert.equal(items[0].output, outA);
    assert.equal(items[1].output, outB);
  });

  it('shows a result whose call is not in the list as a row of its own', () => {
    const o = output('gone');
    const items = groupTranscript([o]);
    assert.equal(items.length, 1);
    assert.equal(items[0].kind, 'tool');
    assert.equal(items[0].call, null);
    assert.equal(items[0].output, o);
  });

  it('does not fold a second result into a call that already has one', () => {
    const items = groupTranscript([call('a'), output('a'), output('a')]);
    assert.equal(items.length, 2);
    assert.equal(items[1].call, null);
  });

  it('drops what the operator never sees', () => {
    const items = groupTranscript([
      stored({ type: 'system_directive', content: 'retry the call' }),
      text('user', 'hi'),
    ]);
    assert.equal(items.length, 1);
    assert.equal(items[0].kind, 'message');
  });

  it('keeps reasoning and subagent results as messages, in order', () => {
    const reasoning = stored({ type: 'reasoning', content: 'hmm' });
    const sub = stored({ type: 'subagent_result', name: 'plan', output: '{"status":"done"}' });
    const items = groupTranscript([reasoning, sub]);
    assert.deepEqual(
      items.map((i) => i.message),
      [reasoning, sub],
    );
  });
});

describe('mergeResolved: where a decision sits in the transcript', () => {
  const at = (s) => new Date(Date.UTC(2026, 9, 22, 10, 0, 0) + s * 1000).toISOString();
  const decision = (requestId, seconds) => ({ requestId, at: at(seconds), outcome: 'approved' });
  const msg = (seconds) => text('user', `at ${seconds}`);

  it('goes before the first item that is newer than it', () => {
    const items = groupTranscript([
      { ...msg(0), timestamp: at(0) },
      { ...msg(10), timestamp: at(10) },
      { ...msg(20), timestamp: at(20) },
    ]);
    const merged = mergeResolved(items, [decision('r1', 15)]);
    assert.deepEqual(
      merged.map((i) => i.kind),
      ['message', 'message', 'resolved', 'message'],
    );
  });

  it('goes last when nothing is newer', () => {
    const items = groupTranscript([{ ...msg(0), timestamp: at(0) }]);
    const merged = mergeResolved(items, [decision('r1', 5)]);
    assert.equal(merged.at(-1).kind, 'resolved');
  });

  it('places a tool row by the time of its call', () => {
    const c = { ...call('a'), timestamp: at(30) };
    const o = { ...output('a'), timestamp: at(31) };
    const merged = mergeResolved(groupTranscript([c, o]), [decision('r1', 29)]);
    assert.deepEqual(
      merged.map((i) => i.kind),
      ['resolved', 'tool'],
    );
  });

  it('keeps several decisions in the order they were made', () => {
    const items = groupTranscript([{ ...msg(100), timestamp: at(100) }]);
    const merged = mergeResolved(items, [decision('later', 20), decision('earlier', 10)]);
    assert.deepEqual(
      merged.filter((i) => i.kind === 'resolved').map((i) => i.resolved.requestId),
      ['earlier', 'later'],
    );
  });

  it('returns the same list when there is nothing to merge, and never mutates its input', () => {
    const items = groupTranscript([text('user', 'hi')]);
    assert.equal(mergeResolved(items, []), items);
    mergeResolved(items, [decision('r1', 0)]);
    assert.equal(items.length, 1);
  });
});

describe('formatMessageTime', () => {
  const now = new Date(2026, 9, 22, 15, 0, 0);

  it('is just the time for a message of today', () => {
    const label = formatMessageTime(new Date(2026, 9, 22, 9, 5, 0).toISOString(), now);
    assert.ok(!label.includes(','));
    assert.match(label, /9|09/);
  });

  it('adds the date for another day of this year', () => {
    const label = formatMessageTime(new Date(2026, 9, 21, 9, 5, 0).toISOString(), now);
    assert.ok(label.includes(','));
    assert.ok(!label.includes('2026'));
  });

  it('adds the year once it is not this one', () => {
    const label = formatMessageTime(new Date(2025, 11, 31, 9, 5, 0).toISOString(), now);
    assert.ok(label.includes('2025'));
  });

  it('is null for a missing or bad timestamp', () => {
    assert.equal(formatMessageTime(null, now), null);
    assert.equal(formatMessageTime('nope', now), null);
  });
});

describe('summarizeArgs', () => {
  it('joins the first two plain values', () => {
    assert.equal(summarizeArgs({ device: 'uav_1', n: 6, nested: { a: 1 }, third: 'x' }), 'uav_1 · 6');
  });

  it('skips nested values and empty strings', () => {
    assert.equal(summarizeArgs({ route: [1, 2], note: '' }), '');
  });

  it('cuts a long value', () => {
    const summary = summarizeArgs({ q: 'x'.repeat(100) });
    assert.equal(summary.length, 48);
    assert.ok(summary.endsWith('…'));
  });

  it('accepts a string or nothing', () => {
    assert.equal(summarizeArgs('plain'), 'plain');
    assert.equal(summarizeArgs(null), '');
  });
});

describe('durationLabel', () => {
  const at = (s) => new Date(Date.UTC(2026, 9, 22, 10, 0, 0) + s * 1000).toISOString();

  it('formats sub-ten-second, seconds and minutes', () => {
    assert.equal(durationLabel(at(0), at(0.4)), '0.4s');
    assert.equal(durationLabel(at(0), at(12)), '12s');
    assert.equal(durationLabel(at(0), at(65)), '1m 05s');
  });

  it('is null when a time is missing or the order is wrong', () => {
    assert.equal(durationLabel(null, at(1)), null);
    assert.equal(durationLabel(at(5), at(1)), null);
    assert.equal(durationLabel('nope', at(1)), null);
  });
});

describe('paramRows: what the operator approves', () => {
  it('lists every key with its whole value', () => {
    const rows = paramRows({ device: 'uav_1', alt: 40, route: [{ lat: 1, lng: 2 }], hold: false });
    assert.deepEqual(rows, [
      { key: 'device', value: 'uav_1' },
      { key: 'alt', value: '40' },
      { key: 'route', value: '[{"lat":1,"lng":2}]' },
      { key: 'hold', value: 'false' },
    ]);
  });

  it('never cuts a long value', () => {
    const [row] = paramRows({ blob: 'y'.repeat(5000) });
    assert.equal(row.value.length, 5000);
  });

  it('shows a non-object input as one row, and nothing for no input', () => {
    assert.deepEqual(paramRows([1, 2]), [{ key: 'input', value: '[1,2]' }]);
    assert.deepEqual(paramRows(null), []);
  });
});

describe('history: grouping by day, without subagents', () => {
  const now = new Date(2026, 9, 22, 12, 0, 0);
  const at = (daysAgo, hour = 9) => new Date(2026, 9, 22 - daysAgo, hour, 0, 0).toISOString();
  const chat = (id, daysAgo, extra = {}) => ({
    id,
    name: id,
    lastUpdated: at(daysAgo),
    agent: 'default',
    preview: null,
    parentChatId: null,
    ...extra,
  });

  it('puts chats in Today / Yesterday / Earlier, newest first', () => {
    const groups = groupHistory(
      [
        chat('old', 5),
        chat('today-early', 0, { lastUpdated: at(0, 8) }),
        chat('yday', 1),
        chat('today-late', 0, { lastUpdated: at(0, 11) }),
      ],
      '',
      now,
    );
    assert.deepEqual(
      groups.map((g) => g.label),
      ['Today', 'Yesterday', 'Earlier'],
    );
    assert.deepEqual(
      groups[0].chats.map((c) => c.id),
      ['today-late', 'today-early'],
    );
  });

  it('leaves out the groups with nothing in them', () => {
    const groups = groupHistory([chat('a', 0)], '', now);
    assert.deepEqual(
      groups.map((g) => g.label),
      ['Today'],
    );
  });

  it('keeps subagent chats out: they live under their parent', () => {
    const groups = groupHistory([chat('main', 0), chat('sub', 0, { parentChatId: 'main' })], '', now);
    assert.deepEqual(
      groups[0].chats.map((c) => c.id),
      ['main'],
    );
  });

  it('filters by name, preview or agent, ignoring case', () => {
    const chats = [
      chat('dock', 0, { name: 'Dock inspection', preview: 'load the mission' }),
      chat('wind', 0, { name: 'Wind farm', agent: 'planner' }),
    ];
    assert.equal(groupHistory(chats, 'DOCK', now)[0].chats[0].id, 'dock');
    assert.equal(groupHistory(chats, 'mission', now)[0].chats[0].id, 'dock');
    assert.equal(groupHistory(chats, 'planner', now)[0].chats[0].id, 'wind');
    assert.deepEqual(groupHistory(chats, 'zzz', now), []);
  });

  it('does not mutate the list it is given', () => {
    const chats = [chat('b', 1), chat('a', 0)];
    groupHistory(chats, '', now);
    assert.deepEqual(
      chats.map((c) => c.id),
      ['b', 'a'],
    );
  });

  it('treats a bad timestamp as Earlier', () => {
    assert.equal(dayGroupOf('not a date', now), 'Earlier');
    assert.equal(historyTimeLabel('not a date', now), '');
  });
});
