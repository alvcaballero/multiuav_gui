import { describe, it, before, after } from 'node:test';
import assert from 'node:assert/strict';
import sequelize from '../common/sequelize.js';
import { MessageOrchestrator, previewOf } from '../models/chat/chat.js';
import { ChatHistoryManager } from '../models/chat/chatHistoryManager.js';

const item = (message) => ({ message });

describe('previewOf: the line a chat shows in the history list', () => {
  it('is the latest user or assistant text', () => {
    const items = [
      item({ role: 'user', content: 'first', type: 'text' }),
      item({ role: 'assistant', content: 'the answer', type: 'text' }),
    ];
    assert.equal(previewOf(items), 'the answer');
  });

  it('skips tool calls, reasoning and the orchestrator nudges', () => {
    const items = [
      item({ role: 'user', content: 'load the mission' }),
      item({ role: 'assistant', type: 'function_call', name: 'load_mission', arguments: '{}' }),
      item({ role: 'assistant', type: 'reasoning', content: 'thinking…' }),
      item({ role: 'user', type: 'system_directive', content: 'retry the call' }),
    ];
    assert.equal(previewOf(items), 'load the mission');
  });

  it('reads the text block of a message that carries an image', () => {
    const items = [
      item({
        role: 'user',
        content: [
          { type: 'input_text', text: 'what is this?' },
          { type: 'input_image', image_url: 'data:image/png;base64,AAAA' },
        ],
      }),
    ];
    assert.equal(previewOf(items), 'what is this?');
  });

  it('collapses whitespace and cuts a long line', () => {
    const long = `${'word '.repeat(60)}\n\nend`;
    const preview = previewOf([item({ role: 'assistant', content: long })]);
    assert.ok(preview.length <= 120);
    assert.ok(preview.endsWith('…'));
    assert.ok(!preview.includes('\n'));
  });

  it('is null when there is nothing to show', () => {
    assert.equal(previewOf([]), null);
    assert.equal(previewOf([item({ role: 'assistant', type: 'function_call', name: 'x' })]), null);
  });
});

// Real SQL: the list is what the history view and the subagent menu are drawn from.
describe('listChats: parent, subagent count and preview', () => {
  const created = [];

  before(async () => {
    await sequelize.models.Chat.sync();
    await sequelize.models.ChatMessage.sync();
  });

  after(async () => {
    await sequelize.models.ChatMessage.destroy({ where: { chatId: created } });
    await sequelize.models.Chat.destroy({ where: { id: created } });
  });

  const make = async (name, metadata = {}) => {
    const chat = await MessageOrchestrator.createChat(name, metadata);
    created.push(chat.id);
    return chat;
  };

  it('counts the subagents of a chat and points each one at its parent', async () => {
    const parent = await make('parent');
    const first = await make('planner-sub', { parentChatId: parent.id, agentProfile: 'planner' });
    const second = await make('monitor-sub', { parentChatId: parent.id, agentProfile: 'agv' });

    const listed = await MessageOrchestrator.listChats();
    const byId = (id) => listed.find((c) => c.id === id);

    assert.equal(byId(parent.id).subagentCount, 2);
    assert.equal(byId(parent.id).parentChatId, null);
    assert.equal(byId(first.id).parentChatId, parent.id);
    assert.equal(byId(second.id).parentChatId, parent.id);
    assert.equal(byId(first.id).subagentCount, 0);
  });

  it('previews the last text of a top-level chat and none for a subagent', async () => {
    const parent = await make('with-preview');
    const sub = await make('sub', { parentChatId: parent.id });
    await ChatHistoryManager.saveMessage(parent.id, {
      from: 'user',
      message: { role: 'user', content: 'inspect the dock', type: 'text' },
    });
    await ChatHistoryManager.saveMessage(sub.id, {
      from: 'user',
      message: { role: 'user', content: 'plan a route', type: 'text' },
    });

    const listed = await MessageOrchestrator.listChats();
    assert.equal(listed.find((c) => c.id === parent.id).preview, 'inspect the dock');
    assert.equal(listed.find((c) => c.id === sub.id).preview, null);
  });
});
