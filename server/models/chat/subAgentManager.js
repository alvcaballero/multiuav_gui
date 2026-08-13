import { chatLogger } from '../../common/logger.js';
import { eventBus, EVENTS } from '../../common/eventBus.js';
import { ChatHistoryManager } from './chatHistoryManager.js';
import { resolveAgentForChat, setAgentForChat, resolveAgent } from './agents/index.js';
import { MessageOrchestrator } from './chat.js';
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
    const baseContext = `- parent_chat_id: ${parentChatId}\n- secondary_chat_id: ${secondaryChatId}`;
    const fullContext = contextInstructions ? `${baseContext}\n${contextInstructions}` : baseContext;

    const systemPromptContent = `${agentDef.systemPrompt}\n---\nSession_context:\n${fullContext}\nMandatory: Maintain all the session context data accurately and unchanged the session.`;

    await ChatHistoryManager.addMessage(secondaryChatId, 'system', { role: 'system', content: systemPromptContent });

    MessageOrchestrator.processMessage(secondaryChatId, userMessage).catch((error) => {
      chatLogger.error(`[createSubAgent] Background processing failed for ${secondaryChatId}:`, error);
      updateSubAgentStatus(secondaryChatId, 'error', { error: error.message });
      eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, {
        chatId: parentChatId,
        from: 'assistant',
        timestamp: new Date().toISOString(),
        message: {
          role: 'assistant',
          content: `Error en subagente ${agentType}: ${error.message}`,
          type: 'text',
          status: 'error',
        },
      });
    });

    return { secondaryChatId, msg: `Subagent ${agentType} started.` };
  }

  /**
   * Injects the result of a subagent tool call into the main chat and resumes it.
   * Replaces the placeholder tool_result for `toolName` in the main chat history,
   * clears the provider session, and continues the conversation.
   *
   * @param {string} chatId         - Main chat ID
   * @param {string} toolName       - MCP tool name whose placeholder to replace (ignored if subAgentChatId resolves one)
   * @param {string} status         - 'valid' | 'error' | 'incomplete'
   * @param {string} description    - Human-readable summary
   * @param {Object} payload        - Data to embed in the tool result output
   * @param {string} subAgentChatId - Subagent chat id; when given, its registered parentToolName
   *                                  is used instead of the explicit toolName, and its status is
   *                                  marked 'done'
   */
  static async injectSubAgentResponse({ chatId, toolName, status, description, payload = {}, subAgentChatId }) {
    if (subAgentChatId) {
      const subAgent = getSubAgent(subAgentChatId);
      if (subAgent?.parentToolName) {
        toolName = subAgent.parentToolName;
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

    const newOutput = JSON.stringify({
      content: [{ type: 'text', text: JSON.stringify({ status, description, ...payload }) }],
    });
    const newContent = `Tool result [${toolName}] [${status}]: ${description}`;

    const hidden = await ChatHistoryManager.hideAndReplaceToolResult(chatId, toolName, newOutput, newContent);

    if (!hidden) {
      chatLogger.warn(
        `[injectSubAgentResponse] No ${toolName} tool_result found in chat ${chatId} — injecting as new message`
      );
      await ChatHistoryManager.addMessage(chatId, 'assistant', {
        type: 'function_call_output',
        name: toolName,
        output: newOutput,
      });
    }

    await ChatHistoryManager.clearSession(chatId);

    const sessionId = await ChatHistoryManager.getSessionId(chatId);
    const agent = await resolveAgentForChat(chatId);
    const allowedTools = agent.allowedTools;
    const systemInstructions = agent.systemPrompt;
    chatLogger.info(`[injectSubAgentResponse] Agent: ${agent.name}, Tools: ${allowedTools?.join(', ')}`);

    const history = await ChatHistoryManager.loadHistory(chatId);
    const realToolResult = history.findLast(
      (item) => item.message?.type === 'function_call_output' && item.message?.name === toolName
    );
    const toolResultForLLM = realToolResult?.message ?? {
      type: 'function_call_output',
      name: toolName,
      output: newOutput,
    };

    // Emit the replaced function_call + tool_result via WebSocket
    const callId = realToolResult?.message?.call_id;
    if (callId) {
      const pairedFunctionCall = history.findLast(
        (item) =>
          (item.message?.type === 'function_call' || item.message?.type === 'tool_call') &&
          (item.message?.call_id === callId || item.message?.id === callId)
      );
      if (pairedFunctionCall) MessageOrchestrator.emitAssistantMessage(pairedFunctionCall);
    }
    if (realToolResult) MessageOrchestrator.emitAssistantMessage(realToolResult);

    if (subAgentChatId) updateSubAgentStatus(subAgentChatId, 'done');

    MessageOrchestrator.continueAfterTools(
      [toolResultForLLM],
      chatId,
      sessionId,
      systemInstructions,
      allowedTools,
      false,
      agent,
      true
    )
      .then(({ output }) => {
        const hasToolCalls = output.some((item) => item.type === 'function_call' || item.type === 'tool_call');
        if (hasToolCalls) {
          const tools = MessageOrchestrator.getToolsForProvider(allowedTools);
          return MessageOrchestrator.handleToolCallsLoop(
            output,
            tools,
            chatId,
            sessionId,
            systemInstructions,
            allowedTools,
            agent
          );
        }
      })
      .catch((err) => {
        chatLogger.error(`[injectSubAgentResponse] continueAfterTools failed for chat ${chatId}:`, err);
        eventBus.emitSafe(EVENTS.CHAT_ASSISTANT_MESSAGE, {
          chatId,
          from: 'assistant',
          timestamp: new Date().toISOString(),
          message: {
            role: 'assistant',
            content: `Error al procesar resultado del subagente: ${err.message}`,
            type: 'text',
            status: 'error',
          },
        });
      });

    return { ok: true, msg: 'Subagent response injected. Main chat processing resumed.' };
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
