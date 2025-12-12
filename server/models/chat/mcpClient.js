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
    this.reconnectDelay = 2000; // 2 segundos inicial
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
      // Limpiar cliente anterior si existe
      if (this.client) {
        try {
          await this.client.close();
        } catch (e) {
          // Ignorar errores al cerrar
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
      this.reconnectAttempts = 0; // Resetear intentos al conectar exitosamente
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
   * Intenta reconectar al servidor MCP con backoff exponencial
   */
  async reconnect() {
    if (this.isReconnecting) {
      chatLogger.info('⏳ Ya hay un intento de reconexión en curso...');
      return;
    }

    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      chatLogger.error('❌ Se alcanzó el máximo de intentos de reconexión');
      return false;
    }

    this.isReconnecting = true;
    this.reconnectAttempts++;

    const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1);

    chatLogger.info(`🔄 Intento de reconexión ${this.reconnectAttempts}/${this.maxReconnectAttempts} en ${delay}ms...`);

    await new Promise(resolve => setTimeout(resolve, delay));

    try {
      await this.connect();
      chatLogger.info('✓ Reconexión exitosa al servidor MCP');
      this.isReconnecting = false;
      return true;
    } catch (error) {
      chatLogger.error(`❌ Intento de reconexión ${this.reconnectAttempts} falló:`, error.message);
      this.isReconnecting = false;

      // Intentar reconectar recursivamente si no se alcanzó el máximo
      if (this.reconnectAttempts < this.maxReconnectAttempts) {
        return await this.reconnect();
      }

      return false;
    }
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
      chatLogger.warn('⚠️  Cliente MCP no conectado, intentando reconectar...');
      const reconnected = await this.reconnect();
      if (!reconnected) {
        throw new Error('Cliente MCP no conectado y no se pudo reconectar');
      }
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

      // Detectar errores de conexión y intentar reconectar
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
        chatLogger.warn('⚠️  Detectada desconexión del servidor MCP, intentando reconectar...');
        this.isConnected = false;

        const reconnected = await this.reconnect();
        if (reconnected) {
          // Reintentar la ejecución de la herramienta
          chatLogger.info('🔄 Reintentando ejecución de herramienta después de reconexión...');
          return await this.executeTool(toolName, args);
        }
      }

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

  /**
   * Verifica la salud de la conexión intentando listar las herramientas
   * @returns {Promise<boolean>} true si la conexión está activa
   */
  async checkConnection() {
    if (!this.client || !this.isConnected) {
      return false;
    }

    try {
      await this.client.listTools();
      return true;
    } catch (error) {
      chatLogger.warn('⚠️  Verificación de conexión falló:', error.message);
      this.isConnected = false;
      return false;
    }
  }

  /**
   * Resetea el contador de intentos de reconexión
   * Útil cuando se quiere forzar nuevos intentos después de alcanzar el máximo
   */
  resetReconnectAttempts() {
    this.reconnectAttempts = 0;
    chatLogger.info('🔄 Contador de intentos de reconexión reseteado');
  }

  formatTools() {}




}
export { MCPclient };
