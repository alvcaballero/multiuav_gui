import { describe, it, before, after } from 'node:test';
import assert from 'node:assert/strict';
import sequelize from '../common/sequelize.js';
import { ChatHistoryManager } from '../models/chat/chatHistoryManager.js';
import { EveOrchestrator, eveRelay } from '../models/chat/eveOrchestrator.js';
import { MessageOrchestrator } from '../models/chat/chat.js';
import { eveEvent, loadEveFixture } from './helpers/eveFixtures.js';

describe('Eve child sessions through the bridge and SQL history', () => {
  const originalClient = eveRelay.client;
  const opens = [];
  let parent;
  let childId;
  let childEvents;

  before(async () => {
    await sequelize.models.Chat.sync();
    await sequelize.models.ChatMessage.sync();
    parent = await ChatHistoryManager.createChat('new Eve parent', {
      engine: 'eve',
      eveAgent: 'assistant',
      sessionId: 'wrun_parent',
    });
    childEvents = loadEveFixture('child-session');
    eveRelay.client = {
      async *streamEvents(agent, sessionId, { startIndex }) {
        opens.push({ agent, sessionId, startIndex });
        const events =
          sessionId === 'wrun_parent'
            ? [
                eveEvent('turn.started'),
                // Workflow delegation: no subagent-call action is needed for discovery.
                eveEvent('agent.started', {
                  sessionId: 'wrun_child',
                  name: 'planner_sandbox',
                  callId: 'c1',
                  taskId: 't1',
                }),
                eveEvent('message.completed', { message: 'Parent answer' }),
                eveEvent('turn.completed'),
                eveEvent('session.waiting'),
              ]
            : childEvents;
        for (const event of events.slice(startIndex)) yield event;
      },
    };
  });

  after(async () => {
    eveRelay.stopAll();
    eveRelay.client = originalClient;
    await sequelize.close();
  });

  it('discovers and reads the child independently using the parent endpoint', async () => {
    await eveRelay.ensure(parent.id);
    const children = await EveOrchestrator.listChildSessions(parent.id);
    assert.equal(children.length, 1);
    childId = children[0].chatId;
    await eveRelay.running.get(childId)?.promise;
    const metadata = await ChatHistoryManager.getChatMetadata(childId);
    assert.equal(metadata.engine, 'eve');
    assert.equal(metadata.parentChatId, parent.id);
    assert.equal(metadata.eveAgent, 'assistant');
    assert.equal(metadata.eveAgentName, 'planner_sandbox');
    assert.equal(metadata.sessionId, 'wrun_child');
    assert.equal(metadata.eveStreamIndex, childEvents.length);
    assert.ok(opens.some((o) => o.agent === 'assistant' && o.sessionId === 'wrun_child'));

    const history = await MessageOrchestrator.getHistory(childId);
    const brief = childEvents.find((e) => e.type === 'message.received').data.message;
    assert.equal(history.messages.filter((m) => m.from === 'user').length, 1);
    assert.equal(history.messages[0].message.content, brief);
    assert.ok(history.messages.some((m) => m.from === 'assistant'));
    const parentHistory = await MessageOrchestrator.getHistory(parent.id);
    assert.deepEqual(
      parentHistory.messages.map((m) => m.message.content),
      ['Parent answer']
    );
  });

  it('preserves getOrCreateChat defaults and resolves simultaneous requests for one new identity', async () => {
    const id = `${parent.id}_shared`;
    const results = await Promise.all([
      ChatHistoryManager.getOrCreateChat(id, { name: 'shared', engine: 'eve' }),
      ChatHistoryManager.getOrCreateChat(id, { name: 'shared', engine: 'eve' }),
    ]);
    assert.equal(results[0].id, results[1].id);
    const existing = await ChatHistoryManager.getOrCreateChat(id, { name: 'replacement', engine: 'legacy' });
    assert.equal(existing.name, 'shared');
    assert.deepEqual(existing.metadata, { name: 'shared', engine: 'eve' });
    assert.equal(await sequelize.models.Chat.count({ where: { id } }), 1);
  });

  it('repeated discovery preserves identity, cursor, turn and the delegated brief', async () => {
    const before = await ChatHistoryManager.getChatMetadata(childId);
    const child = await EveOrchestrator.attachChildSession(parent.id, {
      sessionId: 'wrun_child',
      name: 'planner_sandbox',
      callId: 'c1',
      taskId: 't1',
    });
    await eveRelay.running.get(child.id)?.promise;
    assert.equal(child.id, childId);
    assert.deepEqual(await ChatHistoryManager.getChatMetadata(childId), before);
    const brief = childEvents.find((e) => e.type === 'message.received').data.message;
    await ChatHistoryManager.saveEveChildBrief(childId, brief);
    assert.equal((await MessageOrchestrator.getHistory(childId)).messages.filter((m) => m.from === 'user').length, 1);
    const listed = await MessageOrchestrator.listChats();
    assert.equal(listed.find((c) => c.id === parent.id).subagentCount, 1);
    assert.equal(listed.find((c) => c.id === childId).agent, 'planner_sandbox');
  });

  it('resumes an attached but unread child and reports its durable status', async () => {
    await ChatHistoryManager.updateChat(childId, { metadata: { eveTurn: null, eveStreamIndex: 0 } });
    await EveOrchestrator.resumeChat(childId);
    await eveRelay.running.get(childId)?.promise;
    assert.equal((await EveOrchestrator.listChildSessions(parent.id))[0].status, 'done');
    await ChatHistoryManager.updateChat(childId, { metadata: { eveTurn: { phase: 'awaiting_input' } } });
    assert.equal((await EveOrchestrator.listChildSessions(parent.id))[0].status, 'waiting');
    await ChatHistoryManager.updateChat(childId, { metadata: { eveTurn: { phase: 'failed' } } });
    assert.equal((await EveOrchestrator.listChildSessions(parent.id))[0].status, 'error');
  });
});
