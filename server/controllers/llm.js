import { MessageOrchestrator} from '../models/llm/llm.js';


export class llmController {
  static initializeLLMProvider(provider, apiKey) {
    MessageOrchestrator.initializeLLMProvider(provider, apiKey);
  }
  
  static async sendMessage(req, res) {
    const { message } = req.body;

    if (!message) {
      return res.status(400).json({ error: 'Message is required.' });
    }

    if (!MessageOrchestrator.isReady()) {
      console.error('LLM Provider not initialized.');
      return res.status(500).json({ error: 'AI service not ready.' });
    }

    try {
        const aiResponse = await MessageOrchestrator.processMessage(message);
        res.json({ ...aiResponse });
    } catch (error) {
      console.error('Error in chatController.sendMessage:', error);
      res.status(500).json({ error: error.message || 'Failed to get AI response.' });
    }
  }
}
