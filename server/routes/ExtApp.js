import { Router } from 'express';

import { ExtAppController } from '../controllers/ExtApp.js';

export const ExtAppRouter = Router();

ExtAppRouter.get('/updateToken', ExtAppController.UpdateToken);
ExtAppRouter.post('/start', ExtAppController.missionStart);
ExtAppRouter.post('/result', ExtAppController.missionResult);
ExtAppRouter.post('/media', ExtAppController.missionMedia);
