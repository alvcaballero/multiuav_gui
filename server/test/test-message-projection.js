import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import {
  projectMessage,
  normalizeRole,
  normalizeType,
  MESSAGE_ROLES,
  MESSAGE_TYPES,
  CONTENT_MAX_LENGTH,
} from '../models/chat/messageProjection.js';

describe('projectMessage', () => {
  it('projects a tool call onto assistant/tool_call', () => {
    const result = projectMessage({ type: 'function_call', name: 'create_mission_plan', call_id: 'c1' });

    assert.deepEqual(result, { role: 'assistant', type: 'tool_call', content: 'Tool call: create_mission_plan' });
  });

  it('treats the tool_call alias identically to function_call', () => {
    const a = projectMessage({ type: 'function_call', name: 'get_devices' });
    const b = projectMessage({ type: 'tool_call', name: 'get_devices' });

    assert.deepEqual(a, b);
  });

  it('projects a tool result onto tool/tool_result and truncates the output', () => {
    const output = 'x'.repeat(CONTENT_MAX_LENGTH + 250);
    const result = projectMessage({ type: 'function_call_output', call_id: 'c1', output });

    assert.equal(result.role, 'tool');
    assert.equal(result.type, 'tool_result');
    assert.equal(result.content.length, CONTENT_MAX_LENGTH);
  });

  it('leaves content null when a tool result output is not a string', () => {
    const result = projectMessage({ type: 'function_call_output', output: { plan: [] } });

    assert.equal(result.type, 'tool_result');
    assert.equal(result.content, null);
  });

  it('flattens Responses API output_text blocks', () => {
    const result = projectMessage({
      type: 'message',
      role: 'assistant',
      content: [
        { type: 'output_text', text: 'Plan ready.' },
        { type: 'output_text', text: '3 waypoints.' },
      ],
    });

    assert.deepEqual(result, { role: 'assistant', type: 'text', content: 'Plan ready.\n3 waypoints.' });
  });

  it('ignores non-text blocks when flattening', () => {
    const result = projectMessage({
      type: 'message',
      role: 'assistant',
      content: [
        { type: 'output_text', text: 'See image' },
        { type: 'input_image', image_data: 'base64...' },
      ],
    });

    assert.equal(result.content, 'See image');
  });

  it('defaults a message with no role to assistant', () => {
    const result = projectMessage({ type: 'message', content: 'hi' });

    assert.equal(result.role, 'assistant');
  });

  it('projects reasoning summaries', () => {
    const result = projectMessage({
      type: 'reasoning',
      summary: [{ text: 'Checked collisions.' }, { text: 'Chose UAV 2.' }],
    });

    assert.deepEqual(result, {
      role: 'assistant',
      type: 'reasoning',
      content: 'Checked collisions.\nChose UAV 2.',
    });
  });

  it('projects a bare Gemini text part, which previously fell through as unknown/null', () => {
    const result = projectMessage({ type: 'text', text: 'I will retrieve the turbine details.' });

    assert.deepEqual(result, {
      role: 'assistant',
      type: 'text',
      content: 'I will retrieve the turbine details.',
    });
  });

  it('projects a plain {role, content} user message', () => {
    const result = projectMessage({ role: 'user', content: 'Inspect the wind farm' });

    assert.deepEqual(result, { role: 'user', type: 'text', content: 'Inspect the wind farm' });
  });

  it('projects multimodal user input as multipart, keeping only the text', () => {
    const result = projectMessage({
      role: 'user',
      content: [
        { type: 'input_text', text: 'What is in this photo?' },
        { type: 'input_image', image_data: 'base64...' },
      ],
    });

    assert.equal(result.role, 'user');
    assert.equal(result.type, 'multipart');
    // input_text is not a projected block kind; images never reach `content`.
    assert.equal(result.content, null);
  });

  it('coerces an unknown role to `unknown` instead of writing it through', () => {
    const result = projectMessage({ role: 'planner', content: 'done' });

    assert.equal(result.role, 'unknown');
  });

  it('never silently blanks an unrecognised payload type', () => {
    const result = projectMessage({ type: 'web_search_call', role: 'assistant', content: 'searching' });

    assert.equal(result.type, 'unknown');
    assert.equal(result.role, 'assistant');
    assert.equal(result.content, 'searching');
  });

  it('projects null/non-object payloads as fully unknown', () => {
    assert.deepEqual(projectMessage(null), { role: 'unknown', type: 'unknown', content: null });
    assert.deepEqual(projectMessage('a string'), { role: 'unknown', type: 'unknown', content: null });
  });

  it('always returns values the ENUM columns accept', () => {
    const payloads = [
      { type: 'function_call', name: 'f' },
      { type: 'function_call_output', output: 'o' },
      { type: 'message', role: 'assistant', content: 'm' },
      { type: 'reasoning', summary: [] },
      { role: 'user', content: 'u' },
      { type: 'mystery_type' },
      null,
    ];

    for (const payload of payloads) {
      const { role, type } = projectMessage(payload);
      assert.ok(MESSAGE_ROLES.includes(role), `role "${role}" is not a valid ENUM member`);
      assert.ok(MESSAGE_TYPES.includes(type), `type "${type}" is not a valid ENUM member`);
    }
  });
});

describe('normalizeRole / normalizeType', () => {
  it('passes through valid members', () => {
    assert.equal(normalizeRole('tool'), 'tool');
    assert.equal(normalizeType('tool_result'), 'tool_result');
  });

  it('coerces anything else to unknown', () => {
    assert.equal(normalizeRole('planner'), 'unknown');
    assert.equal(normalizeRole(undefined), 'unknown');
    assert.equal(normalizeType('function_call'), 'unknown');
  });
});
