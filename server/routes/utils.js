import { Router } from 'express';

import { utilsController } from '../controllers/utils.js';

export const utilsRouter = Router();

utilsRouter.get('/datetime', utilsController.getDateTime);
