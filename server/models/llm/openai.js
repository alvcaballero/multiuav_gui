import OpenAI from "openai";
import { LLMProvider } from "./interface.js";
class OpenAIProvider extends LLMProvider {
    constructor(apiKey) {
        super();
        if (!apiKey) {
            throw new Error("OpenAI API Key is required for OpenAIProvider.");
        }
        this.client = new OpenAI({ apiKey });
    }

    async sendMessage(prompt) {
        try {
            const response = await this.client.chat.completions.create({
                model: "gpt-4.1",
                messages: [{ role: "user", content: prompt }],
            });
            console.log(response.output_text);

            return response.choices[0].message.content;
        } catch (error) {
            console.error("Error communicating with OpenAI API:", error);
            throw new Error("Failed to get response from AI model.");
        }
    }
}
export { OpenAIProvider };
