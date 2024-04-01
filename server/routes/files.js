import { Router } from 'express';
import { CreateController } from '../controllers/files.js';

export const createFilesRouter = ({ model }) => {
  const filesRouter = Router();
  const filesController = CreateController({ model });

  filesRouter.get('/get', filesController.getFiles);
  filesRouter.get('/listFiles', filesController.listFiles);
  filesRouter.get('/download/:filename(*)', filesController.donwload);
  filesRouter.post('/test', filesController.testfile);
  filesRouter.post('/TempResult', filesController.MetadataTempImage);
  filesRouter.post('/ProcessImage', filesController.ProcessThermalImages);
  return filesRouter;
};
