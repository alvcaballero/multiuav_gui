import { MessageOrchestrator } from '../models/chat/chat.js';
import logger from '../common/logger.js';

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

    try {
      const history = await MessageOrchestrator.getHistory(chatId);

      // Transform history to client format, excluding system messages
      const messages = history
        .filter((msg) => msg.message?.role !== 'system' && msg.from !== 'system')
        .map((msg) => ({
          from: msg.from,
          message: msg.message,
          timestamp: msg.timestamp || new Date().toISOString(),
        }));

      res.json({ chatId, messages, count: messages.length });
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
  static async buildMissionPlanXYZ(req, res) {
    const { target_elements } = req.body;

    if (!target_elements) {
      return res.status(400).json({ error: 'target_elements is required.' });
    }

    try {
      const aiResponse = await MessageOrchestrator.buildMissionPlanXYZ(req.body);
      res.json({ ...aiResponse });
    } catch (error) {
      logger.error('Error in chatController.buildMissionPlanXYZ:', error);
      res.status(500).json({ error: error.message || 'Failed to create mission plan.' });
    }
  }

  static async verificationMission(req, res) {
    const { origin_global, chat_id } = req.body;

    if (!origin_global || !chat_id) {
      return res.status(400).json({ error: 'Mission data with origin_global and chat_id are required.' });
    }

    try {
      const aiResponse = await MessageOrchestrator.verificationMission(req.body);
      res.json({ ...aiResponse });
    } catch (error) {
      logger.error('Error in chatController.verificationMission:', error);
      res.status(500).json({ error: error.message || 'Failed to verify mission.' });
    }
  }
}
