import { randomUUID } from 'node:crypto';
import { chatLogger } from '../../common/logger.js';
import { ChatHistoryManager } from './chatHistoryManager.js';
import { resolveAgentForChat, setAgentForChat, resolveAgent } from './agents/index.js';
import { MessageOrchestrator } from './chat.js';
import { buildSystemPrompt } from './turnContext.js';
import { emitAssistantError } from './chatEvents.js';
import { registerSubAgent, getSubAgent, updateSubAgentStatus, listSubAgentsForParent } from './subAgentRegistry.js';

export class SubAgentManager {
  /**
   * Creates a subagent chat for background processing.
   *
   * @param {string} parentChatId - ID of the parent chat that initiated this request
   * @param {string} agentType  - Agent type to assign (e.g. 'planner')
   * @param {string} userMessage - First message to send to the subagent
   * @param {string} parentToolName - Name of the tool call in the parent chat this subagent answers to
   * @param {string} contextInstructions - Optional free-text extra context injected into the system prompt
   * @param {Object} contextParams - Optional fixed structured context, injected into the subagent's tool call args
   */
  static async createSubAgent({
    parentChatId,
    agentType,
    userMessage,
    contextInstructions = '',
    contextParams = {},
    parentToolName,
  }) {
    if (!userMessage) throw new Error('userMessage is required');
    if (!parentChatId) throw new Error('parentChatId is required');
    if (!agentType) throw new Error('agentType is required');
    if (!parentToolName) throw new Error('parentToolName is required');

    const secondaryChat = await MessageOrchestrator.createChat(`${agentType.toUpperCase()}-${parentChatId}`);
    const secondaryChatId = secondaryChat.id;
    chatLogger.info(`[createSubAgent] parentChat=${parentChatId} subAgent=${secondaryChatId} type=${agentType}`);

    await setAgentForChat(secondaryChatId, agentType);

    registerSubAgent({
      chatId: secondaryChatId,
      parentChatId,
      agentType,
      parentToolName,
      firstMessage: userMessage,
      contextParams,
    });

    const agentDef = resolveAgent(agentType);
    const systemPromptContent = buildSystemPrompt({
      systemPrompt: agentDef.systemPrompt,
      contextLines: [`parent_chat_id: ${parentChatId}`],
      // contextInstructions is free-form text the caller already formatted
      extraContext: contextInstructions,
      trailer: 'Mandatory: Maintain all the session context data accurately and unchanged the session.',
    });

    await ChatHistoryManager.addMessage(secondaryChatId, 'system', { role: 'system', content: systemPromptContent });

    MessageOrchestrator.processMessage(secondaryChatId, userMessage).catch((error) => {
      chatLogger.error(`[createSubAgent] Background processing failed for ${secondaryChatId}:`, error);
      updateSubAgentStatus(secondaryChatId, 'error', { error: error.message });
      emitAssistantError(parentChatId, `Error en subagente ${agentType}: ${error.message}`);
    });

    return { secondaryChatId, msg: `Subagent ${agentType} started.` };
  }

  /**
   * Injects the result of a subagent into the parent chat and resumes it.
   *
   * A subagent answers minutes after the parent's tool pair already closed, so the
   * result is APPENDED as a new `subagent_result` message rather than back-patched
   * into the original function_call_output. Appending keeps the history immutable
   * (no hiding, no re-inserting, no rewriting the past) and makes N subagents in
   * flight work for free — each one is just one more append.
   *
   * @param {string} chatId         - Parent chat ID
   * @param {string} toolName       - MCP tool name this result answers to (ignored if subAgentChatId resolves one)
   * @param {string} status         - 'valid' | 'error' | 'incomplete'
   * @param {string} description    - Human-readable summary
   * @param {Object} payload        - Data to embed in the result output
   * @param {string} subAgentChatId - Subagent chat id; when given, its registered parentToolName
   *                                  is used instead of the explicit toolName, and its status is
   *                                  marked 'done'
   */
  static async injectSubAgentResponse({ chatId, toolName, status, description, payload = {}, subAgentChatId }) {
    let agentName = null;
    if (subAgentChatId) {
      const subAgent = getSubAgent(subAgentChatId);
      if (subAgent?.parentToolName) {
        toolName = subAgent.parentToolName;
        agentName = subAgent.agentType ?? null;
      } else {
        chatLogger.warn(
          `[injectSubAgentResponse] subAgentChatId ${subAgentChatId} not found in registry — falling back to passed toolName`
        );
      }
    }

    chatLogger.info(`[injectSubAgentResponse] chat=${chatId} tool=${toolName} status=${status}`);

    const chatExists = await ChatHistoryManager.chatExists(chatId);
    if (!chatExists) {
      const err = new Error(`Chat not found: ${chatId}`);
      err.statusCode = 404;
      throw err;
    }

    // Same convention as function_call_output: `output` is a JSON STRING, never an
    // object. Everything the subagent controls (description, payload) lives inside
    // that JSON, so it can never forge the envelope fields the handlers render.
    const subagentResult = {
      type: 'subagent_result',
      from: 'subagent',
      subAgentChatId: subAgentChatId ?? null,
      agentName,
      name: toolName,
      output: JSON.stringify({ status, description, ...payload }),
    };

    const chatItem = await ChatHistoryManager.addMessage(chatId, 'subagent', subagentResult);
    MessageOrchestrator.emitAssistantMessage(chatItem);

    await ChatHistoryManager.clearSession(chatId);

    const sessionId = await ChatHistoryManager.getSessionId(chatId);
    const agent = await resolveAgentForChat(chatId);
    const allowedTools = agent.allowedTools;
    const systemInstructions = agent.systemPrompt;
    chatLogger.info(`[injectSubAgentResponse] Agent: ${agent.name}, Tools: ${allowedTools?.join(', ')}`);

    if (subAgentChatId) updateSubAgentStatus(subAgentChatId, 'done');

    // Resuming the parent chat is a new billable turn: everything the parent spends
    // digesting the subagent's answer is grouped under this id.
    const turnId = randomUUID();

    // The result is already persisted above and replayed from history — passing it
    // again as toolOutputs would deliver it twice to the provider.
    MessageOrchestrator.continueAfterTools(
      [],
      chatId,
      sessionId,
      systemInstructions,
      allowedTools,
      false,
      agent,
      true,
      { turnId, phase: 'subagent_resume', iteration: 0 }
    )
      .then(({ output }) => {
        const hasToolCalls = output.some((item) => item.type === 'function_call' || item.type === 'tool_call');
        if (hasToolCalls) {
          return MessageOrchestrator.runToolLoop(
            output,
            chatId,
            {
              tools: MessageOrchestrator.getToolsForProvider(allowedTools),
              sessionId,
              systemInstructions,
              allowedTools,
              agent,
            },
            { turnId }
          );
        }
      })
      .catch((err) => {
        chatLogger.error(`[injectSubAgentResponse] continueAfterTools failed for chat ${chatId}:`, err);
        emitAssistantError(chatId, `Error al procesar resultado del subagente: ${err.message}`);
      });

    return { ok: true, msg: 'Subagent response injected. Parent chat processing resumed.' };
  }

  /**
   * Lists the runtime state of every subagent created by a given parent chat.
   * @param {string} parentChatId
   * @returns {Array<Object>}
   */
  static listSubAgents(parentChatId) {
    return listSubAgentsForParent(parentChatId);
  }
}
