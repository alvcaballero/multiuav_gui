import { Router } from 'express';

import { filesController } from '../controllers/files.js';

export const filesRouter = Router();

filesRouter.get('/download', filesController.donwload);
