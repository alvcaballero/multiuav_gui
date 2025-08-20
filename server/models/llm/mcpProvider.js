// https://medium.com/google-cloud/model-context-protocol-mcp-with-google-gemini-llm-a-deep-dive-full-code-ea16e3fac9a3

import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StdioClientTransport } from '@modelcontextprotocol/sdk/client/stdio.js';
import { StreamableHTTPClientTransport } from '@modelcontextprotocol/sdk/client/streamableHttp.js';
import { SSEClientTransport } from '@modelcontextprotocol/sdk/client/sse.js';

import { MCPconfig } from '../../config/config.js';

class MCPclient {
  constructor(llm) {
    this.llm;
    this.transport;
    this.tools = [];
    this.llm = llm;

    this.client = new Client({
      name: 'example-client',
      version: '1.0.0',
    });
  }

  async connectStdio() {
    this.transport = new StdioClientTransport({
      command: 'npx',
      args: ['-y', 'tsx', '/home/grvc/mcpServers/muav_gui_assistant/src/index.ts', 'stdio'],
    });
    await this.client.connect(this.transport);
  }
  async connectHttp() {
    this.transport = new StreamableHTTPClientTransport(new URL(baseUrl));
    await this.client.connect(transport);
  }
  async connectSSE() {
    const sseTransport = new SSEClientTransport(baseUrl);
    await this.client.connect(sseTransport);
  }

  async connect() {
    switch (MCPconfig.transport) {
      case 'stdio':
        await this.connectStdio();
        break;
      case 'http':
        await this.connectHttp();
        break;
      case 'sse':
        await this.connectSSE();
        break;
      default:
        throw new Error(`Unknown transport: ${MCPconfig.transport}`);
    }
  }

  async listTools() {
    //console.log("Listing tools...");
    let tools = await this.client.listTools();
    this.tools = tools['tools'];
    //console.log("Tools:", this.tools);
  }

  async listPrompts() {
    return await this.client.listPrompts();
  }

  async listResources() {
    return await this.client.listResources();
  }

  async disconnect() {
    await this.client.disconnect();
  }
  async callTool(name, args) {
    const tool = this.tools.find((t) => t.name === name);
    if (!tool) throw new Error(`Tool not found: ${name}`);
    return tool.call(args);
  }

  async getPrompt(name, args) {
    const prompt = await this.client.getPrompt({ name, arguments: args });
    if (!prompt) throw new Error(`Prompt not found: ${name}`);
    return prompt;
  }

  async readResource(uri) {
    const resource = await this.client.readResource({ uri });
    if (!resource) throw new Error(`Resource not found: ${uri}`);
    return resource;
  }

  formatTools() {}

  async sendMessage(message) {
    if (!this.client) {
      throw new Error('MCP client is not initialized.');
    }
    if (!this.llm) {
      throw new Error('LLM provider is not initialized.');
    }
    if (this.tools.length === 0) {
      console.log('No tools available, fetching tools...');
      // Fetch tools if not already fetched
      await this.listTools();
    }

    const response = await this.llm.sendMessage(message, this.tools);
    if (!response) {
      throw new Error('Failed to get response from LLM.');
    }
    console.log('LLM response:', response);
    return response;
    for (const content of response.contents) {
      if (content.type === 'text') {
        return content.text;
      } else if (content.type === 'tool_call') {
        const toolResponse = await this.callTool(content.name, content.arguments);
        return toolResponse;
      }
    }
  }
}
export { MCPclient };
