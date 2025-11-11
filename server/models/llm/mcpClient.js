// https://medium.com/google-cloud/model-context-protocol-mcp-with-google-gemini-llm-a-deep-dive-full-code-ea16e3fac9a3

import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StdioClientTransport } from '@modelcontextprotocol/sdk/client/stdio.js';
import { StreamableHTTPClientTransport } from '@modelcontextprotocol/sdk/client/streamableHttp.js';
import { SSEClientTransport } from '@modelcontextprotocol/sdk/client/sse.js';
import logger, { chatLogger } from '../../common/logger.js';

import { MCPconfig } from '../../config/config.js';

class MCPclient {
  constructor() {
    this.transport = null;
    this.tools = [];
    this.isConnected = false;
    this.client = null;
  }

  async connectStdio() {
    this.transport = new StdioClientTransport({
      command: 'npx',
      args: ['-y', 'tsx', '/home/grvc/mcpServers/muav_gui_assistant/src/index.ts', 'stdio'],
    });
  }
  async connectHttp() {
    logger.info('Connecting via HTTP to', MCPconfig.url);
    this.transport = new StreamableHTTPClientTransport(new URL(MCPconfig.url));
  }
  async connectSSE() {
    this.transport = new SSEClientTransport(new URL(MCPconfig.url));
  }

  async connect() {
    logger.info('Connecting to MCP server using', MCPconfig, 'transport...');

    try {
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

      if (this.transport == null) {
        throw new Error('Transport not initialized');
      }

      this.client = new Client(
        {
          name: 'muav-mcp-client',
          version: '1.0.0',
        },
        {
          capabilities: {
            tools: {},
          },
        }
      );

      await this.client.connect(this.transport);

      this.isConnected = true;
      await this.loadTools();

      chatLogger.info('✓ Conectado al servidor MCP');
      chatLogger.info(`✓ ${this.tools.length} herramientas cargadas`);

      return true;
    } catch (error) {
      chatLogger.error('Error connecting to MCP server:', error);
      this.isConnected = false;
      throw error;
    }
  }

  async loadTools() {
    if (!this.client) {
      throw new Error('Cliente MCP no inicializado');
    }
    try {
      const response = await this.client.listTools();
      this.tools = response.tools || [];
    } catch (error) {
      console.error('Error cargando herramientas MCP:', error);
      this.tools = [];
    }
  }

  getTools() {
    return this.tools;
  }

  /**
   * Obtiene las herramientas en formato compatible con OpenAI
   * @returns {Array} Array de herramientas en formato OpenAI
   */
  getToolsForOpenAI() {
    return this.tools.map((tool) => ({
      type: 'function',
      function: {
        name: tool.name,
        description: tool.description || '',
        parameters: tool.inputSchema || {
          type: 'object',
          properties: {},
        },
      },
    }));
  }
  /**
   * Obtiene las herramientas en formato compatible con OpenAI
   * @returns {Array} Array de herramientas en formato OpenAI
   */
  getToolsForGemini() {
    return this.tools.map((tool) => ({
      type: 'function',
      function: {
        name: tool.name,
        description: tool.description || '',
        parameters: tool.inputSchema || {
          type: 'object',
          properties: {},
        },
      },
    }));
  }

  /**
   * Obtiene las herramientas en formato compatible con Anthropic
   * @returns {Array} Array de herramientas en formato Anthropic
   */
  getToolsForAnthropic() {
    return this.tools.map((tool) => ({
      name: tool.name,
      description: tool.description || '',
      input_schema: tool.inputSchema || {
        type: 'object',
        properties: {},
      },
    }));
  }

  async listPrompts() {
    return await this.client.listPrompts();
  }

  async listResources() {
    return await this.client.listResources();
  }

  /**
   * Cierra la conexión con el servidor MCP
   */
  async disconnect() {
    if (this.client) {
      await this.client.close();
      this.isConnected = false;
      console.log('✓ Desconectado del servidor MCP');
    }
  }

  /**
   * Ejecuta una herramienta del servidor MCP
   * @param {string} toolName - Nombre de la herramienta
   * @param {Object} args - Argumentos para la herramienta
   * @returns {Promise<any>} Resultado de la ejecución
   */
  async executeTool(toolName, args = {}) {
    if (!this.client || !this.isConnected) {
      throw new Error('Cliente MCP no conectado');
    }
    const tool = this.tools.find((t) => t.name === toolName);
    if (!tool) throw new Error(`Tool not found: ${toolName}`);

    try {
      console.log(`→ Ejecutando herramienta MCP: ${toolName}`);
      console.log(`  Argumentos:`, args);

      const result = await this.client.callTool({
        name: toolName,
        arguments: args,
      });

      console.log(`✓ Herramienta ejecutada: ${toolName}`);
      return result;
    } catch (error) {
      console.error(`Error ejecutando herramienta ${toolName}:`, error);
      throw error;
    }
  }

  /**
   * Verifica si el cliente está conectado
   * @returns {boolean}
   */
  isReady() {
    return this.isConnected && this.tools.length > 0;
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
        content:
          'You are a helpful assistant with access to various tools. Use them when necessary to answer user questions.',
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
