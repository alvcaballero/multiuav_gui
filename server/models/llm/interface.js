export class LLMProvider {
  constructor() {
    if (new.target === LLMProvider) {
      throw new TypeError('Cannot construct LLMProvider instances directly. Use a concrete subclass.');
    }
  }

  /**
   * Envía un mensaje al LLM y obtiene una respuesta.
   * @param {string} prompt - El mensaje del usuario.
   * @returns {Promise<string>} La respuesta del LLM.
   */
  async sendMessage(prompt) {
    throw new Error("Method 'sendMessage()' must be implemented by subclasses.");
  }
}
