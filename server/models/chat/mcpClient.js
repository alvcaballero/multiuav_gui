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
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 5;
    this.reconnectDelay = 2000; // 2 seconds initial
    this.isReconnecting = false;
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
      // Clean up previous client if exists
      if (this.client) {
        try {
          await this.client.close();
        } catch (e) {
          // Ignore errors when closing
        }
      }

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
      this.reconnectAttempts = 0; // Reset attempts on successful connection
      await this.loadTools();

      chatLogger.info('✓ Connected to MCP server');
      chatLogger.info(`✓ ${this.tools.length} tools loaded`);

      return true;
    } catch (error) {
      chatLogger.error('Error connecting to MCP server:', error);
      this.isConnected = false;
      throw error;
    }
  }

  async loadTools() {
    if (!this.client) {
      throw new Error('MCP client not initialized');
    }
    try {
      const response = await this.client.listTools();
      this.tools = response.tools || [];
    } catch (error) {
      chatLogger.error('Error loading MCP tools:', error);
      this.tools = [];
    }
  }

  getTools() {
    return this.tools;
  }

  /**
   * Gets tools in OpenAI compatible format
   * @returns {Array} Array of tools in OpenAI format
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
   * Gets tools in Gemini compatible format
   * @returns {Array} Array of tools in Gemini format
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
   * Gets tools in Anthropic compatible format
   * @returns {Array} Array of tools in Anthropic format
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
   * Attempts to reconnect to MCP server with exponential backoff
   */
  async reconnect() {
    if (this.isReconnecting) {
      chatLogger.info('⏳ Reconnection attempt already in progress...');
      return;
    }

    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      chatLogger.error('❌ Maximum reconnection attempts reached');
      return false;
    }

    this.isReconnecting = true;
    this.reconnectAttempts++;

    const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1);

    chatLogger.info(`🔄 Reconnection attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts} in ${delay}ms...`);

    await new Promise((resolve) => setTimeout(resolve, delay));

    try {
      await this.connect();
      chatLogger.info('✓ Successfully reconnected to MCP server');
      this.isReconnecting = false;
      return true;
    } catch (error) {
      chatLogger.error(`❌ Reconnection attempt ${this.reconnectAttempts} failed:`, error.message);
      this.isReconnecting = false;

      // Try to reconnect recursively if max not reached
      if (this.reconnectAttempts < this.maxReconnectAttempts) {
        return await this.reconnect();
      }

      return false;
    }
  }

  /**
   * Closes the connection to the MCP server
   */
  async disconnect() {
    if (this.client) {
      await this.client.close();
      this.isConnected = false;
      chatLogger.info('✓ Disconnected from MCP server');
    }
  }

  /**
   * Executes a tool from the MCP server
   * @param {string} toolName - Tool name
   * @param {Object} args - Arguments for the tool
   * @returns {Promise<any>} Execution result
   */
  async executeTool(toolName, args = {}) {
    if (!this.client || !this.isConnected) {
      chatLogger.warn('⚠️  MCP client not connected, attempting to reconnect...');
      const reconnected = await this.reconnect();
      if (!reconnected) {
        throw new Error('MCP client not connected and could not reconnect');
      }
    }

    const tool = this.tools.find((t) => t.name === toolName);
    if (!tool) throw new Error(`Tool not found: ${toolName}`);

    try {
      chatLogger.info(`→ Executing MCP tool: ${toolName}`);
      //chatLogger.debug(`  Arguments:`, args);

      const result = await this.client.callTool({
        name: toolName,
        arguments: args,
      });

      chatLogger.info(`✓ Tool executed: ${toolName}`);
      return result;
    } catch (error) {
      chatLogger.error(`Error executing tool ${toolName}:`, error);

      // Detect connection errors and attempt to reconnect
      const isConnectionError =
        error.message?.includes('ECONNREFUSED') ||
        error.message?.includes('ECONNRESET') ||
        error.message?.includes('EPIPE') ||
        error.message?.includes('Connection closed') ||
        error.message?.includes('socket hang up') ||
        error.code === 'ECONNREFUSED' ||
        error.code === 'ECONNRESET' ||
        error.code === 'EPIPE';

      if (isConnectionError) {
        chatLogger.warn('⚠️  MCP server disconnection detected, attempting to reconnect...');
        this.isConnected = false;

        const reconnected = await this.reconnect();
        if (reconnected) {
          // Retry tool execution after reconnection
          chatLogger.info('🔄 Retrying tool execution after reconnection...');
          return await this.executeTool(toolName, args);
        }
      }

      throw error;
    }
  }

  /**
   * Checks if the client is connected
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

  /**
   * Checks connection health by attempting to list tools
   * @returns {Promise<boolean>} true if connection is active
   */
  async checkConnection() {
    if (!this.client || !this.isConnected) {
      return false;
    }

    try {
      await this.client.listTools();
      return true;
    } catch (error) {
      chatLogger.warn('⚠️  Connection check failed:', error.message);
      this.isConnected = false;
      return false;
    }
  }

  /**
   * Resets the reconnection attempts counter
   * Useful when you want to force new attempts after reaching the maximum
   */
  resetReconnectAttempts() {
    this.reconnectAttempts = 0;
    chatLogger.info('🔄 Reconnection attempts counter reset');
  }

  formatTools() {}
}
export { MCPclient };
