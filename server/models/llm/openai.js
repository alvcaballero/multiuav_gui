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

  // output: [
  //   {
  //     id: 'rs_0a25fdebf18fe8b100691221769f9c81908fc6f07f83bf8334',
  //     type: 'reasoning',
  //     summary: []
  //   },
  //   {
  //     id: 'fc_0a25fdebf18fe8b1006912217793808190bc7089753aaf5440',
  //     type: 'function_call',
  //     status: 'completed',
  //     arguments: '{}',
  //     call_id: 'call_twt3DXArJadNMa7H9mQwlqos',
  //     name: 'get_devices'
  //   }
  // ],

  async sendMessage(messages, tools = []) {
    // Si messages es un string, convertirlo a formato de mensajes
    if (typeof messages === 'string') {
      const initMsg = {
        role: 'system',
        content: SystemPrompts['openai'],
      };
      messages = [initMsg, { role: 'user', content: messages }];
    }

    let parseTools = [];
    if (tools && tools.length > 0) {
      parseTools = this.convertToolsForMCP(tools);
      //console.log('parseTools', parseTools);
    }

    try {
      const response = await this.client.responses.create({
        model: 'gpt-5',
        input: messages,
        tools: parseTools,
      });

      console.log('OpenAI Response:', JSON.stringify(response, null, 2));

      // Retornar la respuesta completa para que pueda ser procesada
      return {
        output: response.output,
        usage: response.usage,
      };
    } catch (error) {
      console.error('Error communicating with OpenAI API:', error);
      throw new Error('Failed to get response from AI model.');
    }
  }
}
export { OpenAIProvider };
