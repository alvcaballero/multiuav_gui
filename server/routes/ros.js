import { Router } from 'express';

import { rosController } from '../controllers/ros.js';

export const rosRouter = Router();

rosRouter.get('/topics', rosController.getTopics);
