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
    console.log('Connecting via HTTP to', MCPconfig.url);
    this.transport = new StreamableHTTPClientTransport(new URL(MCPconfig.url));
    await this.client.connect(this.transport);
  }
  async connectSSE() {
    const sseTransport = new SSEClientTransport(new URL(MCPconfig.url));
    await this.client.connect(sseTransport);
  }

  async connect() {
    console.log('Connecting to MCP server using', MCPconfig, 'transport...');
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

    console.log(`Calling tool: ${name} with args:`, args);

    // Usar el cliente MCP para llamar a la herramienta
    const result = await this.client.callTool({
      name: name,
      arguments: args,
    });

    console.log(`Tool ${name} result:`, result);
    return result;
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

  async sendMessage(userMessage) {
    if (!this.client) {
      throw new Error('MCP client is not initialized.');
    }
    if (!this.llm) {
      throw new Error('LLM provider is not initialized.');
    }
    if (this.tools.length === 0) {
      console.log('No tools available, fetching tools...');
      await this.listTools();
    }

    // Inicializar el historial de conversación
    const conversationHistory = [
      {
        role: 'system',
        content: 'You are a helpful assistant with access to various tools. Use them when necessary to answer user questions.',
      },
      {
        role: 'user',
        content: userMessage,
      },
    ];

    const maxIterations = 10;
    let iteration = 0;

    while (iteration < maxIterations) {
      iteration++;
      console.log(`\n=== Iteration ${iteration} ===`);

      // Enviar mensajes al LLM
      const response = await this.llm.sendMessage(conversationHistory, this.tools);

      if (!response || !response.output) {
        throw new Error('Failed to get response from LLM.');
      }

      console.log('LLM response output:', JSON.stringify(response.output, null, 2));

      // Procesar cada elemento de la respuesta
      let hasToolCalls = false;
      let textResponse = '';

      for (const content of response.output) {
        if (content.type === 'text') {
          textResponse += content.text || '';
        } else if (content.type === 'function_call' || content.type === 'tool_call') {
          hasToolCalls = true;

          // Agregar el mensaje del asistente con la llamada a la herramienta
          conversationHistory.push({
            role: 'assistant',
            content: '',
            tool_calls: [
              {
                id: content.call_id || content.id,
                type: 'function',
                function: {
                  name: content.name,
                  arguments: JSON.stringify(content.arguments || {}),
                },
              },
            ],
          });

          // Ejecutar la herramienta
          try {
            const toolResult = await this.callTool(content.name, content.arguments || {});

            // Agregar el resultado de la herramienta a la conversación
            conversationHistory.push({
              role: 'tool',
              tool_call_id: content.call_id || content.id,
              content: JSON.stringify(toolResult),
            });
          } catch (error) {
            console.error(`Error calling tool ${content.name}:`, error);
            // Agregar el error a la conversación
            conversationHistory.push({
              role: 'tool',
              tool_call_id: content.call_id || content.id,
              content: JSON.stringify({ error: error.message }),
            });
          }
        } else if (content.type === 'reasoning') {
          console.log('Reasoning:', content.summary);
        }
      }

      // Si no hay llamadas a herramientas y hay texto, retornar la respuesta
      if (!hasToolCalls && textResponse) {
        console.log('Final response:', textResponse);
        return textResponse;
      }

      // Si no hay llamadas a herramientas y no hay texto, algo salió mal
      if (!hasToolCalls && !textResponse) {
        throw new Error('LLM did not provide a valid response.');
      }

      // Si hay llamadas a herramientas, continuar el loop
    }

    throw new Error(`Max iterations (${maxIterations}) reached without final response.`);
  }
}
export { MCPclient };
