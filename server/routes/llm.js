import { Router } from 'express';
import { llmController } from '../controllers/llm.js';

export const llmRouter = Router();

llmRouter.post('/', llmController.sendMessage);
