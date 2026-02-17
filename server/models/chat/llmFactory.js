import { OpenAIHandler } from './openaiHandler.js';
import { GeminiHandler } from './geminiHandler.js';
import { AnthropicHandler } from './antropicHandler.js';
import { OllamaHandler } from './ollamaHandler.js';
import { SystemPrompts } from './prompts/index.js';

/**
 * Factory para crear instancias de manejadores de LLM
 */
export class LLMFactory {
  /**
   * Crea un manejador de LLM según el proveedor especificado
   * @param {string} provider - Nombre del proveedor (openai, anthropic, gemini)
   * @param {string} apiKey - API key del proveedor
   * @param {string} model - Modelo a utilizar
   * @returns {BaseLLMHandler} Instancia del manejador
   */
  static createHandler(provider, apiKey, model) {
    switch (provider.toLowerCase()) {
      case 'openai':
        return new OpenAIHandler(apiKey, model, SystemPrompts.main);
      case 'gemini':
        return new GeminiHandler(apiKey, model, SystemPrompts.main);
      case 'anthropic':
      case 'claude':
        return new AnthropicHandler(apiKey, model, SystemPrompts.main);
      case 'ollama':
        return new OllamaHandler(apiKey, model, SystemPrompts.main);
      default:
        throw new Error(`Proveedor de LLM no soportado: ${provider}`);
    }
  }

  /**
   * Lista de proveedores soportados
   * @returns {string[]}
   */
  static getSupportedProviders() {
    return ['openai', 'anthropic', 'claude', 'gemini', 'ollama'];
  }
}
