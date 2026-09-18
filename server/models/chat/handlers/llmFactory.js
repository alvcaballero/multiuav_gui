import { OpenAIHandler } from './openaiHandler.js';
import { GeminiHandler } from './geminiHandler.js';
import { AnthropicHandler } from './antropicHandler.js';
import { OpenAICompatibleHandler } from './openaiCompatibleHandler.js';
import { COMPATIBLE_PROVIDERS, isCompatibleProvider, normalizeBaseURL } from './compatibleProviders.js';
import { SystemPrompts } from '../agents/index.js';

/**
 * Factory para crear instancias de manejadores de LLM.
 *
 * Tres transportes, no uno por proveedor:
 *  - Responses API      → OpenAIHandler (solo OpenAI; nadie más la implementa)
 *  - Chat Completions   → OpenAICompatibleHandler (ollama, llamacpp, vLLM, LM Studio…)
 *  - APIs propietarias  → GeminiHandler, AnthropicHandler
 *
 * Añadir un servidor compatible es una entrada en `compatibleProviders.js`, no una clase.
 */
export class LLMFactory {
  /**
   * Crea un manejador de LLM según el proveedor especificado
   * @param {string} provider - Nombre del proveedor
   * @param {string} apiKey - API key del proveedor (ignorada por servidores locales)
   * @param {string} [model] - Modelo a utilizar; si falta se usa el default del proveedor
   * @param {object} [options] - Opciones extra para proveedores compatibles
   * @param {string} [options.baseURL] - Endpoint Chat Completions (sobreescribe el default del preset)
   * @param {object} [options.capabilities] - Overrides de request-shaping
   * @returns {BaseLLMHandler} Instancia del manejador
   */
  static createHandler(provider, apiKey, model, options = {}) {
    const name = provider.toLowerCase();

    if (isCompatibleProvider(name)) {
      const preset = COMPATIBLE_PROVIDERS[name];

      // The endpoint comes from `baseURL` (LLM_*_BASE_URL) and nothing else; `apiKey` is
      // purely a token here, empty for local servers. A URL in the token slot is rejected
      // at boot (see server.js) rather than silently re-interpreted, so a stale
      // LLM_OLLAMA_API_KEY pointing at a REMOTE host can never be read as "localhost".
      const baseURL = normalizeBaseURL(options.baseURL || preset.defaultBaseURL);

      return new OpenAICompatibleHandler(apiKey || 'not-needed', model || preset.defaultModel, SystemPrompts.main, {
        baseURL,
        providerName: name,
        capabilities: { ...preset.capabilities, ...(options.capabilities || {}) },
      });
    }

    switch (name) {
      case 'openai':
        return new OpenAIHandler(apiKey, model, SystemPrompts.main);
      case 'gemini':
        return new GeminiHandler(apiKey, model, SystemPrompts.main);
      case 'anthropic':
      case 'claude':
        return new AnthropicHandler(apiKey, model, SystemPrompts.main);
      default:
        throw new Error(`Proveedor de LLM no soportado: ${provider}`);
    }
  }

  /**
   * Lista de proveedores soportados
   * @returns {string[]}
   */
  static getSupportedProviders() {
    return ['openai', 'anthropic', 'claude', 'gemini', ...Object.keys(COMPATIBLE_PROVIDERS)];
  }
}
