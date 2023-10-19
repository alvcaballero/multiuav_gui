import { Router } from 'express';

import { positionsController } from '../controllers/positions.js';

export const positionsRouter = Router();

positionsRouter.get('/', positionsController.getAll);



