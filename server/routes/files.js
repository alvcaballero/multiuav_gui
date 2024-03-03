import { Router } from 'express';

import { filesController } from '../controllers/files.js';

export const filesRouter = Router();

filesRouter.get('/routes', filesController.getfiles);
filesRouter.get('/download/:filename(*)', filesController.donwload);
filesRouter.post('/test', filesController.testfile);
filesRouter.post('/TempResult', filesController.MetadataTempImage);
filesRouter.post('/ProcessImage', filesController.ProcessThermalImages);
