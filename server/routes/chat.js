import { Router } from 'express';
import { chatController } from '../controllers/chat.js';
import { upload } from '../common/tempfiles.js';
import { SpeechController } from '../controllers/speech.js';

export const chatRouter = Router();

// Legacy HTTP endpoint (keep for compatibility)
chatRouter.post('/', chatController.sendMessage);

// Chat history management endpoints
chatRouter.get('/history/:chatId', chatController.getChatHistory);
chatRouter.get('/chats', chatController.listChats);
chatRouter.post('/chats', chatController.createChat);
chatRouter.delete('/chats/:chatId', chatController.deleteChat);
chatRouter.patch('/chats/:chatId', chatController.renameChat);
chatRouter.post('/chats/:chatId/fork', chatController.forkChat);

chatRouter.post('/subagents', chatController.createSubAgent);
chatRouter.get('/subagents/:parentChatId', chatController.getSubAgentsStatus);
chatRouter.post('/subagents/:chatId/inject', chatController.injectSubAgentResponse);
chatRouter.post('/test_mcp_tool', chatController.testMcpTool);
// Ruta para speech-to-text (transcripción)
chatRouter.post('/stt', upload.single('audio'), SpeechController.speechToText);
// Ruta para text-to-speech (síntesis de voz)
chatRouter.post('/tts', SpeechController.textToSpeech);
