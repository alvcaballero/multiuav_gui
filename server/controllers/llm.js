import { getLLMProvider } from '../models/llm/provider.js';
import { MCPclient } from '../models/llm/mcp.js';
import {MCPenable} from  '../config/config.js';
let llmProviderInstance;
let MCPclientInstance;
export function initializeLLMProvider(provider,apiKey) {
  if (!llmProviderInstance) {
    // Puedes cambiar 'gemini' por 'openai' si implementas ese proveedor.
    llmProviderInstance = getLLMProvider(provider, apiKey);
    console.log('LLM Provider initialized:', provider );
  }
  if (MCPenable && !MCPclientInstance) {
    // Si MCP está habilitado, inicializamos el cliente MCP con el proveedor LLM.
    MCPclientInstance = new MCPclient(llmProviderInstance);
    MCPclientInstance.connect()
      .then(() => {
        console.log('MCP Client connected successfully.');
      })
      .catch(error => {
        console.error('Error connecting MCP Client:', error);
      });
    console.log('MCP Client initialized with LLM Provider.');
    } else if (!MCPenable) {
    console.log('MCP is disabled, using direct LLM provider.');
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
      if (MCPenable) {
        // Si MCP está habilitado, usamos MCPclient
        const aiResponse = await MCPclientInstance.sendMessage(message);
        res.json({ reply: aiResponse });
      } else {
        // Si MCP no está habilitado, usamos el proveedor LLM directamente
        const aiResponse = await llmProviderInstance.sendMessage(message);
        res.json({ reply: aiResponse });
      }
    } catch (error) {
      console.error('Error in chatController.sendMessage:', error);
      res.status(500).json({ error: error.message || 'Failed to get AI response.' });
    }
  }
}
