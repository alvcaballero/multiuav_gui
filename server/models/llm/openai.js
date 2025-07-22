import OpenAI from 'openai';
import { LLMProvider } from './interface.js';
import { SystemPrompts } from './SystemPromps.js';

class OpenAIProvider extends LLMProvider {
  constructor(apiKey) {
    super();
    if (!apiKey) {
      throw new Error('OpenAI API Key is required for OpenAIProvider.');
    }
    this.client = new OpenAI({ apiKey });
  }


  convertToolsForMCP(tools) {
    return tools.map((tool) => {
      return {
        type: 'function',
        name: tool.name,
        description: tool.description,
        parameters: tool.inputSchema,
      };
    });
  }

  async sendMessage(prompt, tools = []) {
    const initMsg = {
      role: 'developer',
      content: SystemPrompts['openai'],
    };
    let parseTools = [];
    if (tools && tools.length > 0) {
      parseTools = this.convertToolsForMCP(tools);
      //console.log('parseTools', parseTools);
    }
    try {
      //const response = await this.client.chat.completions.create({
      const response = await this.client.responses.create({
        model: 'gpt-4.1',
        input: [initMsg, { role: 'user', content: prompt }],
        tools: parseTools,
      });
      console.log(response);

      return response.output_text;
    } catch (error) {
      console.error('Error communicating with OpenAI API:', error);
      throw new Error('Failed to get response from AI model.');
    }
  }
}
export { OpenAIProvider };
