import { chatLogger } from '../../common/logger.js';
import { ChatHistoryManager } from './chatHistoryManager.js';
import { setAgentForChat, resolveAgent } from './agents/index.js';
import { MessageOrchestrator } from './chat.js';
import { buildSystemPrompt } from './turnContext.js';
import { emitAssistantError } from './chatEvents.js';
import { registerSubAgent, getSubAgent, updateSubAgentStatus, listSubAgentsForParent } from './subAgentRegistry.js';

/**
 * Resolves subagent info by chatId: in-memory registry first (fast path, the
 * common case since the registry is populated for the lifetime of the run),
 * falling back to the chat's persisted metadata when the registry doesn't
 * have it (e.g. server restarted since the subagent was created).
 * @param {string} chatId - Subagent's own chat id
 * @returns {Promise<Object>} { chatId, parentChatId, agentType, parentToolName, contextParams }
 */
async function getSubAgentInfo(chatId) {
  const registered = getSubAgent(chatId);
  if (registered) {
    const { parentChatId, agentType, parentToolName, contextParams } = registered;
    return { chatId, parentChatId, agentType, parentToolName, contextParams: contextParams || {} };
  }

  const chatExists = await ChatHistoryManager.chatExists(chatId);
  if (!chatExists) {
    const err = new Error(`Chat not found: ${chatId}, possibly wrong chatId try again with correct chatId`);
    err.statusCode = 404;
    throw err;
  }

  const chatMetadata = await ChatHistoryManager.getChatMetadata(chatId);
  return {
    chatId,
    parentChatId: chatMetadata.parentChatId,
    agentType: chatMetadata.agentType,
    parentToolName: chatMetadata.parentToolName,
    contextParams: chatMetadata.contextParams || {},
  };
}

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

    const subAgentChat = await MessageOrchestrator.createChat(`${agentType.toUpperCase()}-${parentChatId}`, {
      parentChatId,
      parentToolName,
      contextParams,
    });
    const subAgentChatId = subAgentChat.id;
    chatLogger.info(`[createSubAgent] parentChat=${parentChatId} subAgent=${subAgentChatId} type=${agentType}`);

    await setAgentForChat(subAgentChatId, agentType);

    registerSubAgent({
      chatId: subAgentChatId,
      parentChatId,
      agentType,
      parentToolName,
      firstMessage: userMessage,
      contextParams,
    });

    const agentDef = resolveAgent(agentType);
    const systemPromptContent = buildSystemPrompt({
      systemPrompt: agentDef.systemPrompt,
      contextLines: [`session_chat_id: ${subAgentChatId}`],
      // contextInstructions is free-form text the caller already formatted
      extraContext: contextInstructions,
      trailer: 'Mandatory: Maintain all the session context data accurately and unchanged the session.',
    });

    await ChatHistoryManager.addMessage(subAgentChatId, 'system', { role: 'system', content: systemPromptContent });

    MessageOrchestrator.processMessage(subAgentChatId, userMessage).catch((error) => {
      chatLogger.error(`[createSubAgent] Background processing failed for ${subAgentChatId}:`, error);
      updateSubAgentStatus(subAgentChatId, 'error', { error: error.message });
      emitAssistantError(parentChatId, `Error en subagente ${agentType}: ${error.message}`);
    });

    return { secondaryChatId: subAgentChatId, msg: `Subagent ${agentType} started.` };
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
   * @param {string} chatId         - subAgent chat ID
   * @param {string} toolName       - MCP tool name this result answers to (ignored if the registry resolves one)
   * @param {string} status         - 'valid' | 'error' | 'incomplete'
   * @param {string} description    - Human-readable summary
   * @param {Object} payload        - Data to embed in the result output
   */
  static async injectSubAgentResponse({ chatId, toolName, status, description, payload = {} }) {
    const { parentChatId, agentType, parentToolName } = await getSubAgentInfo(chatId);
    if (parentToolName) toolName = parentToolName;

    chatLogger.info(`[injectSubAgentResponse] chat=${chatId} tool=${toolName} status=${status}`);

    // Same convention as function_call_output: `output` is a JSON STRING, never an
    // object. Everything the subagent controls (description, payload) lives inside
    // that JSON, so it can never forge the envelope fields the handlers render.
    const subagentResult = {
      type: 'subagent_result',
      from: 'subagent',
      subAgentChatId: chatId ?? null,
      agentName: agentType ?? null,
      name: toolName,
      output: JSON.stringify({ status, description, ...payload }),
    };
    // output: { "content": [{ "type": "text", "text": "..." }] }

    updateSubAgentStatus(chatId, 'done');

    // Goes through the public entry point on purpose: this arrives asynchronously
    // from outside (MCP over HTTP), so it must take the parent chat's lock and
    // queue behind whatever turn is running instead of writing over it.
    // Persisting the message, resuming the LLM and running any follow-up tools
    // are all `processMessage`'s job — nothing to duplicate here.
    MessageOrchestrator.processMessage(parentChatId, { kind: 'subagent_result', message: subagentResult }).catch(
      (err) => {
        chatLogger.error(`[injectSubAgentResponse] resume failed for chat ${parentChatId}:`, err);
        emitAssistantError(parentChatId, `Error al procesar resultado del subagente: ${err.message}`);
      }
    );

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
