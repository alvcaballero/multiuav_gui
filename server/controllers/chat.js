import { MessageOrchestrator} from '../models/chat/chat.js';


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
      console.error('LLM Provider not initialized.');
      return res.status(500).json({ error: 'AI service not ready.' });
    }

    try {
        const aiResponse = await MessageOrchestrator.processMessage({ chatId: effectiveChatId, message });
        res.json({ ...aiResponse });
    } catch (error) {
      console.error('Error in chatController.sendMessage:', error);
      res.status(500).json({ error: error.message || 'Failed to get AI response.' });
    }
  }
  
  static async processMessage(data) {
    console.log('Processing message with data:', data);
    const { chatId, message, timestamp } = data;


    const effectiveChatId = chatId || 'default_http'; // Default for HTTP requests

    return MessageOrchestrator.processMessage(effectiveChatId, message);
  }

  static async getChatHistory(req, res) {
    const { chatId } = req.params;

    try {
      const history = MessageOrchestrator.getHistory(chatId);

      // Transform history to client format
      const messages = history.map(msg => ({
        role: msg.role,
        text: msg.content || msg.text,
        type: msg.type || 'text',
        timestamp: msg.timestamp || new Date().toISOString(),
      }));

      res.json({ chatId, messages });
    } catch (error) {
      console.error('Error getting chat history:', error);
      res.status(500).json({ error: error.message });
    }
  }

  static async listChats(req, res) {
    try {
      const chats = MessageOrchestrator.listChats();
      res.json({ chats });
    } catch (error) {
      console.error('Error listing chats:', error);
      res.status(500).json({ error: error.message });
    }
  }

  static async deleteChat(req, res) {
    const { chatId } = req.params;

    try {
      MessageOrchestrator.resetHistory(chatId);
      res.json({ success: true, chatId });
    } catch (error) {
      console.error('Error deleting chat:', error);
      res.status(500).json({ error: error.message });
    }
  }
}
