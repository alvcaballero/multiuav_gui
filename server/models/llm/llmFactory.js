// https://medium.com/google-cloud/model-context-protocol-mcp-with-google-gemini-llm-a-deep-dive-full-code-ea16e3fac9a3


import { OpenAIHandler } from './openaiHandler.js';
import { GeminiHandler } from './geminiHandler.js';
// import { AnthropicHandler } from './anthropicHandler.js';
import { SystemPrompts } from './SystemPromps.js';
/**
 * Factory para crear instancias de manejadores de LLM
 */
export class LLMFactory {
  /**
   * Crea un manejador de LLM según el proveedor especificado
   * @param {string} provider - Nombre del proveedor (openai, anthropic, etc.)
   * @param {string} apiKey - API key del proveedor
   * @param {string} model - Modelo a utilizar
   * @returns {BaseLLMHandler} Instancia del manejador
   */
  static createHandler(provider, apiKey, model ) {
    switch (provider.toLowerCase()) {
      case 'openai':
        return new OpenAIHandler(apiKey, model, SystemPrompts.openai);
      case 'gemini':
        return new GeminiHandler(apiKey, model, SystemPrompts.gemini);
      // case 'anthropic':
      // case 'claude':
      //   return new AnthropicHandler(apiKey, model);
      
      default:
        throw new Error(`Proveedor de LLM no soportado: ${provider}`);
    }
  }

  /**
   * Lista de proveedores soportados
   * @returns {string[]}
   */
  static getSupportedProviders() {
    return ['openai', 'anthropic','claude', 'gemini'];
  }
}