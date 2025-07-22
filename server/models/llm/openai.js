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

  /*
    {
        "id": "mcpr_682d498e3bd4819196a0ce1664f8e77b04ad1e533afccbfa",
        "type": "mcp_approval_request",
        "arguments": "{\"repoName\":\"modelcontextprotocol/modelcontextprotocol\",\"question\":\"What transport protocols are supported in the 2025-03-26 version of the MCP spec?\"}",
        "name": "ask_question",
        "server_label": "deepwiki"
    }
    */

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
