import { describe, it, before, after } from 'node:test';
import assert from 'node:assert/strict';
import sequelize from '../common/sequelize.js';
import { MessageOrchestrator } from '../models/chat/chat.js';
import { DEFAULT_AGENT } from '../models/chat/agents/index.js';
import { EveDefaultAgent } from '../config/config.js';

// Real SQL: `agent` is read out of chat.metadata, which is what the header of the chat shows.
describe('which agent answers a chat (the header label)', () => {
  const created = [];

  before(async () => {
    await sequelize.models.Chat.sync();
    await sequelize.models.ChatMessage.sync();
  });

  after(async () => {
    await sequelize.models.Chat.destroy({ where: { id: created } });
  });

  const make = async (name, metadata) => {
    const chat = await MessageOrchestrator.createChat(name, metadata);
    created.push(chat.id);
    return chat;
  };

  it('a legacy chat reports its agent profile, and "default" when none was chosen', async () => {
    const plain = await make('plain', {});
    const planner = await make('planner', { agentProfile: 'planner' });

    assert.equal(plain.agent, DEFAULT_AGENT);
    assert.equal(planner.agent, 'planner');

    const listed = await MessageOrchestrator.listChats();
    assert.equal(listed.find((c) => c.id === plain.id).agent, DEFAULT_AGENT);
    assert.equal(listed.find((c) => c.id === planner.id).agent, 'planner');
  });

  it('an eve chat reports the eve agent it was created for, or the default agent', async () => {
    const monitor = await make('monitor', { engine: 'eve', eveAgent: 'monitor' });
    const unnamed = await make('unnamed', { engine: 'eve' });

    const listed = await MessageOrchestrator.listChats();
    assert.equal(listed.find((c) => c.id === monitor.id).agent, 'monitor');
    assert.equal(listed.find((c) => c.id === unnamed.id).agent, EveDefaultAgent);
    assert.equal(listed.find((c) => c.id === monitor.id).engine, 'eve');
  });

  it('knows no model until a handler exists (the header then falls back to the provider alone)', () => {
    assert.equal(MessageOrchestrator.getLLMModel(), null);
  });
});
