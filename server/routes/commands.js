import { Router } from 'express';

import { commandController } from '../controllers/devices.js';

export const devicesRouter = Router();

devicesRouter.get('/', devicesController.getAll);
devicesRouter.post('/', devicesController.create);

devicesRouter.get('/:id', devicesController.getById);
devicesRouter.delete('/:id', devicesController.delete);
devicesRouter.patch('/:id');

devicesRouter.get('/type', devicesController.GetDeviceCategory);
