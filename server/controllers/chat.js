import { MessageOrchestrator } from '../models/chat/chat.js';
import { SubAgentManager } from '../models/chat/subAgentManager.js';
import { logger } from '../common/logger.js';

export class chatController {
  static initializeLLMProvider(provider, apiKey) {
    MessageOrchestrator.initializeLLMProvider(provider, apiKey);
  }

  static async sendMessage(req, res) {
    const { message, chatId } = req.body;
    const effectiveChatId = chatId || 'default_http'; // Default for HTTP requests

    if (!message) {
      return res.status(400).json({ error: 'Message is required.' });
    }

    if (!MessageOrchestrator.isReady()) {
      logger.error('LLM Provider not initialized.');
      return res.status(500).json({ error: 'AI service not ready.' });
    }

    try {
      const aiResponse = await MessageOrchestrator.processMessage(effectiveChatId, message);
      res.json({ ...aiResponse });
    } catch (error) {
      logger.error('Error in chatController.sendMessage:', error);
      res.status(500).json({ error: error.message || 'Failed to get AI response.' });
    }
  }

  static async processMessage(data) {
    logger.debug('Processing message with data:', data);
    const { chatId, message } = data;

    let effectiveChatId = chatId;

    // If no chatId provided, create a new chat on the server
    if (!chatId) {
      const newChat = await MessageOrchestrator.createChat();
      effectiveChatId = newChat.id;
      logger.info(`Created new chat for message: ${effectiveChatId}`);
    }

    // Process message and return result with chatId
    const result = await MessageOrchestrator.processMessage(effectiveChatId, message);
    return { ...result, chatId: effectiveChatId };
  }

  static async getChatHistory(req, res) {
    const { chatId } = req.params;
    const { limit, before } = req.query;

    try {
      const { messages: history, hasMore } = await MessageOrchestrator.getHistory(chatId, {
        limit: limit ? parseInt(limit, 10) : undefined,
        before: before || null,
      });

      // Transform history to client format, excluding system messages
      const messages = history
        .filter((msg) => msg.message?.role !== 'system' && msg.from !== 'system')
        .map((msg) => ({
          from: msg.from,
          message: msg.message,
          timestamp: msg.timestamp || new Date().toISOString(),
        }));

      res.json({ chatId, messages, count: messages.length, hasMore });
    } catch (error) {
      logger.error('Error getting chat history:', error);
      res.status(500).json({ error: error.message });
    }
  }

  static async listChats(_req, res) {
    try {
      const chats = await MessageOrchestrator.listChats();
      res.json({ chats });
    } catch (error) {
      logger.error('Error listing chats:', error);
      res.status(500).json({ error: error.message });
    }
  }

  static async createChat(req, res) {
    const { name } = req.body;

    try {
      const chat = await MessageOrchestrator.createChat(name);
      res.status(201).json(chat);
    } catch (error) {
      logger.error('Error creating chat:', error);
      res.status(500).json({ error: error.message });
    }
  }

  /**
   * GET /api/chat/chats/:chatId/usage
   * Token usage at three levels: `requests` (one entry per LLM call — the raw
   * numbers), `turns` (one per user message) and `totals` (whole chat).
   * Query: ?detail=totals | turns | full (default). Use a narrower detail on
   * long chats, where `requests` can be hundreds of entries.
   */
  static async getChatUsage(req, res) {
    const { chatId } = req.params;
    const { detail = 'full' } = req.query;

    try {
      const { totals, turns, requests } = await MessageOrchestrator.getUsage(chatId);
      res.json({
        chatId,
        totals,
        turnCount: turns.length,
        requestCount: requests.length,
        ...(detail === 'totals' ? {} : { turns }),
        ...(detail === 'full' ? { requests } : {}),
      });
    } catch (error) {
      logger.error('Error getting chat usage:', error);
      res.status(500).json({ error: error.message });
    }
  }

  static async deleteChat(req, res) {
    const { chatId } = req.params;
    const { hard } = req.query;

    try {
      await MessageOrchestrator.deleteChat(chatId, hard === 'true');
      res.json({ success: true, chatId });
    } catch (error) {
      logger.error('Error deleting chat:', error);
      res.status(500).json({ error: error.message });
    }
  }

  static async renameChat(req, res) {
    const { chatId } = req.params;
    const { name } = req.body;

    if (!name) {
      return res.status(400).json({ error: 'Name is required.' });
    }

    try {
      const success = await MessageOrchestrator.renameChat(chatId, name);
      if (success) {
        res.json({ success: true, chatId, name });
      } else {
        res.status(400).json({ error: 'Could not rename chat. Database may be disabled.' });
      }
    } catch (error) {
      logger.error('Error renaming chat:', error);
      res.status(500).json({ error: error.message });
    }
  }

  static async forkChat(req, res) {
    const { chatId } = req.params;
    const { messageTimestamp, name } = req.body;

    if (!messageTimestamp) {
      return res.status(400).json({ error: 'messageTimestamp is required.' });
    }

    try {
      const newChat = await MessageOrchestrator.forkChat(chatId, messageTimestamp, name);
      res.status(201).json(newChat);
    } catch (error) {
      logger.error('Error forking chat:', error);
      res.status(500).json({ error: error.message });
    }
  }
  static async createSubAgent(req, res) {
    const { parentChatId, agentType, userMessage, contextInstructions, contextParams, parentToolName } = req.body;
    if (!parentChatId || !agentType || !userMessage || !parentToolName) {
      return res.status(400).json({ error: 'parentChatId, agentType, userMessage and parentToolName are required.' });
    }
    try {
      const result = await SubAgentManager.createSubAgent({
        parentChatId,
        agentType,
        userMessage,
        contextInstructions,
        contextParams,
        parentToolName,
      });
      res.status(201).json(result);
    } catch (error) {
      logger.error('Error in chatController.createSubAgent:', error);
      res.status(500).json({ error: error.message });
    }
  }

  static async injectSubAgentResponse(req, res) {
    const { toolName, status, description, payload } = req.body;
    const { chatId } = req.params;
    if (!chatId) {
      return res.status(400).json({ error: 'chatId is required.' });
    }
    try {
      const result = await SubAgentManager.injectSubAgentResponse({
        chatId,
        toolName,
        status,
        description,
        payload,
      });
      res.json(result);
    } catch (error) {
      logger.error('Error in chatController.injectSubAgentResponse:', error);
      res.status(error.statusCode || 500).json({ error: error.message });
    }
  }

  static async getSubAgentsStatus(req, res) {
    const { parentChatId } = req.params;
    try {
      const subagents = SubAgentManager.listSubAgents(parentChatId);
      res.json({ subagents });
    } catch (error) {
      logger.error('Error in chatController.getSubAgentsStatus:', error);
      res.status(500).json({ error: error.message });
    }
  }

  static async testMcpTool(req, res) {
    const { toolName, toolArgs } = req.body;

    if (!toolName) {
      return res.status(400).json({ error: 'toolName is required.' });
    }

    try {
      const result = await MessageOrchestrator.testMcpTool(toolName, toolArgs ?? {});
      res.json({ toolName, toolArgs: toolArgs ?? {}, result });
    } catch (error) {
      logger.error('Error in chatController.testMcpTool:', error);
      res.status(500).json({ error: error.message });
    }
  }
}
