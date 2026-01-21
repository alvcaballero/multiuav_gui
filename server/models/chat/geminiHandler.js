import { GoogleGenAI } from '@google/genai';
import { BaseLLMHandler } from './baseLLMhandler.js';
import { SystemPrompts } from './prompts/index.js';

export class GeminiHandler extends BaseLLMHandler {
  constructor(apiKey, model = 'gemini-2.0-flash-001', systemPrompt = SystemPrompts.main) {
    super(apiKey, model, systemPrompt);
    if (!apiKey) {
      throw new Error('Google API Key is required for GoogleGeminiProvider.');
    }
    this.genAI = new GoogleGenAI(apiKey);
  }

  convertToolsForMCP(tools) {
    return [
      {
        functionDeclarations: tools.map((tool) => {
          return {
            name: tool.name,
            description: tool.description,
            parameters: {
              type: tool.inputSchema.type,
              properties: tool.inputSchema.properties,
            },
          };
        }),
      },
    ];
  }

  async sendMessage(prompt, tools = []) {
    const initMsg = {
      role: 'developer',
      content: SystemPrompts.main,
    };
    let parseTools = [];
    if (tools && tools.length > 0) {
      parseTools = this.convertToolsForMCP(tools);
    }
    try {
      const response = await this.genAI.models.generateContent({
        model: 'gemini-2.0-flash-001',
        contents: prompt,
        config: {
          systemInstruction: initMsg.content,
          tools: parseTools,
        },
      });
      const text = response.text();
      return text;
    } catch (error) {
      console.error('Error communicating with Google Gemini API:', error);
      throw new Error('Failed to get response from AI model.');
    }
  }
}
