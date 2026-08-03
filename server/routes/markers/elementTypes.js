import { Router } from 'express';
import { elementTypesController } from '../../controllers/markers/elementTypes.js';

export const elementTypesRouter = Router();

elementTypesRouter.get('/', elementTypesController.getAll);
elementTypesRouter.get('/:id', elementTypesController.getById);
elementTypesRouter.post('/', elementTypesController.create);
elementTypesRouter.put('/:id', elementTypesController.update);
elementTypesRouter.delete('/:id', elementTypesController.delete);
