import { Router } from 'express';
import { CreateController } from '../controllers/devices.js';

export const createDevicesRouter = ({ model }) => {
  const devicesRouter = Router();
  const devicesController = CreateController({ model });
  devicesController.getAllDevices();
  devicesRouter.get('/', devicesController.getAll);
  devicesRouter.post('/', devicesController.create);

  devicesRouter.get('/:id', devicesController.getById);
  devicesRouter.delete('/:id', devicesController.delete);
  devicesRouter.patch('/:id');
  return devicesRouter;
};
