import { Router } from 'express';

import { deviceModelsController } from '../controllers/deviceModels.js';

export const deviceModelsRouter = Router();

deviceModelsRouter.get('/', deviceModelsController.getAll);
deviceModelsRouter.get('/:model', deviceModelsController.get);
