import { GoogleGenAI } from '@google/genai';
import { LLMProvider } from './interface.js';

class GoogleGeminiProvider extends LLMProvider {
  constructor(apiKey) {
    super();
    if (!apiKey) {
      throw new Error('Google API Key is required for GoogleGeminiProvider.');
    }
    this.genAI = new GoogleGenAI(apiKey);
  }

  async sendMessage(prompt) {
    try {
      const response = await this.genAI.models.generateContent({ model: 'gemini-2.0-flash-001', contents: prompt });
      const text = response.text();
      return text;
    } catch (error) {
      console.error('Error communicating with Google Gemini API:', error);
      throw new Error('Failed to get response from AI model.');
    }
  }
}
export { GoogleGeminiProvider };
