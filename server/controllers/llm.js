import { getLLMProvider } from '../models/llm/provider.js';

let llmProviderInstance;

export function initializeLLMProvider(provider,apiKey) {
  if (!llmProviderInstance) {
    // Puedes cambiar 'gemini' por 'openai' si implementas ese proveedor.
    llmProviderInstance = getLLMProvider(provider, apiKey);
    console.log('LLM Provider initialized:', provider );
  }
}

export class llmController {
  static async sendMessage(req, res) {
    const { message } = req.body;

    if (!message) {
      return res.status(400).json({ error: 'Message is required.' });
    }

    if (!llmProviderInstance) {
      console.error('LLM Provider not initialized.');
      return res.status(500).json({ error: 'AI service not ready.' });
    }

    try {
      const aiResponse = await llmProviderInstance.sendMessage(message);
      res.json({ reply: aiResponse });
    } catch (error) {
      console.error('Error in chatController.sendMessage:', error);
      res.status(500).json({ error: error.message || 'Failed to get AI response.' });
    }
  }
}
