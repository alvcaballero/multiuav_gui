import { GoogleGeminiProvider } from './gemini.js';
// import { OpenAIChatGPTProvider } from './openai.js';

const Providers = { GEMINI: 'gemini', OPENAI: 'openai' };

export const getLLMProvider = (providerType, apiKey) => {
  switch (providerType) {
    case Providers.GEMINI:
      return new GoogleGeminiProvider(apiKey);
    // case 'openai':
    //     return new OpenAIChatGPTProvider(apiKey); // Ejemplo: Añadir otro proveedor
    default:
      throw new Error(`Unsupported LLM provider type: ${providerType}`);
  }
};
