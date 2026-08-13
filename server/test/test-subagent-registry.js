import { describe, it, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import {
  registerSubAgent,
  getSubAgent,
  getContextParams,
  updateSubAgentStatus,
  listSubAgentsForParent,
  removeSubAgent,
} from '../models/chat/subAgentRegistry.js';

describe('subAgentRegistry', () => {
  const chatId = 'sub-1';
  const parentChatId = 'parent-1';

  beforeEach(() => {
    removeSubAgent(chatId);
    removeSubAgent('sub-2');
  });

  it('registerSubAgent stores a running record with fixed context params', () => {
    const record = registerSubAgent({
      chatId,
      parentChatId,
      agentType: 'planner',
      parentToolName: 'create_mission_plan',
      firstMessage: 'Plan a wind farm inspection',
      contextParams: { global_origin: { lat: 1, lng: 2 } },
    });

    assert.equal(record.status, 'running');
    assert.equal(record.error, null);
    assert.deepEqual(record.contextParams, { global_origin: { lat: 1, lng: 2 } });
    assert.equal(getSubAgent(chatId), record);
  });

  it('contextParams are frozen and cannot be mutated after registration', () => {
    registerSubAgent({
      chatId,
      parentChatId,
      agentType: 'planner',
      parentToolName: 'create_mission_plan',
      firstMessage: 'msg',
      contextParams: { foo: 'bar' },
    });

    const params = getContextParams(chatId);
    assert.throws(() => {
      params.foo = 'changed';
    });
    assert.equal(getContextParams(chatId).foo, 'bar');
  });

  it('getContextParams returns {} for chats that were never registered', () => {
    assert.deepEqual(getContextParams('never-registered'), {});
  });

  it('updateSubAgentStatus transitions status and records an error message', () => {
    registerSubAgent({
      chatId,
      parentChatId,
      agentType: 'planner',
      parentToolName: 'create_mission_plan',
      firstMessage: 'msg',
    });

    const updated = updateSubAgentStatus(chatId, 'error', { error: 'boom' });
    assert.equal(updated.status, 'error');
    assert.equal(updated.error, 'boom');
    assert.equal(getSubAgent(chatId).status, 'error');
  });

  it('updateSubAgentStatus on an unknown chatId returns null without throwing', () => {
    assert.equal(updateSubAgentStatus('unknown-chat', 'done'), null);
  });

  it('listSubAgentsForParent returns only records for the given parent', () => {
    registerSubAgent({
      chatId,
      parentChatId,
      agentType: 'planner',
      parentToolName: 'create_mission_plan',
      firstMessage: 'msg',
    });
    registerSubAgent({
      chatId: 'sub-2',
      parentChatId: 'other-parent',
      agentType: 'agv',
      parentToolName: 'dispatch_agv',
      firstMessage: 'msg2',
    });

    const list = listSubAgentsForParent(parentChatId);
    assert.equal(list.length, 1);
    assert.equal(list[0].chatId, chatId);

    removeSubAgent('sub-2');
  });

  it('removeSubAgent evicts the record', () => {
    registerSubAgent({
      chatId,
      parentChatId,
      agentType: 'planner',
      parentToolName: 'create_mission_plan',
      firstMessage: 'msg',
    });
    removeSubAgent(chatId);
    assert.equal(getSubAgent(chatId), undefined);
  });
});
