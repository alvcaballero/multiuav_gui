import { Router } from 'express';

import { deviceModelsController } from '../controllers/deviceModels.js';

export const deviceModelsRouter = Router();

deviceModelsRouter.get('/', deviceModelsController.getAll);
// Before /:model, or 'capabilities' would be read as a model name.
deviceModelsRouter.get('/capabilities', deviceModelsController.getCapabilities);
deviceModelsRouter.get('/:model', deviceModelsController.get);
