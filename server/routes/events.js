import { Router } from 'express';

import { eventsController } from '../controllers/events.js';

export const eventsRouter = Router();

eventsRouter.get('/', eventsController.getAll);


