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

chatRouter.post('/build_mission_plan_xyz', chatController.buildMissionPlanXYZ);
chatRouter.post('/validate_mission_briefing', chatController.generateMissionBriefing);

// Ruta para speech-to-text (transcripción)
chatRouter.post('/stt', upload.single('audio'), SpeechController.speechToText);
// Ruta para text-to-speech (síntesis de voz)
chatRouter.post('/tts', SpeechController.textToSpeech);
