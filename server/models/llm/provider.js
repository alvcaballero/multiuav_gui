import { GoogleGeminiProvider } from './gemini.js';
import { OpenAIProvider } from './openai.js';

// https://medium.com/google-cloud/model-context-protocol-mcp-with-google-gemini-llm-a-deep-dive-full-code-ea16e3fac9a3

const Providers = { GEMINI: 'gemini', OPENAI: 'openai' };


export const getLLMProvider = (providerType, apiKey) => {
  switch (providerType) {
    case Providers.GEMINI:
      return new GoogleGeminiProvider(apiKey);
    case Providers.OPENAI:
      return new OpenAIProvider(apiKey);
    default:
      throw new Error(`Unsupported LLM provider type: ${providerType}`);
  }
};
