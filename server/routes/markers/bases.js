import { Router } from 'express';
import { basesController } from '../../controllers/markers/bases.js';

export const basesRouter = Router();

basesRouter.get('/', basesController.getAll);
basesRouter.get('/assigned', basesController.getAssigned);
basesRouter.get('/:id', basesController.getById);
basesRouter.post('/', basesController.create);
basesRouter.put('/:id', basesController.update);
basesRouter.delete('/:id', basesController.delete);
