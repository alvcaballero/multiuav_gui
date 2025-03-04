import { Router } from 'express';
import { devicesController } from '../controllers/devices.js';

export const createDevicesRouter = () => {
  const devicesRouter = Router();

  devicesRouter.get('/', devicesController.getAll);
  devicesRouter.post('/', devicesController.create);
  devicesRouter.get('/:id', devicesController.getById);
  devicesRouter.delete('/:id', devicesController.delete);
  devicesRouter.put('/:id', devicesController.update);
  devicesRouter.patch('/:id');
  return devicesRouter;
};
