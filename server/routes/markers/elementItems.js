import { Router } from 'express';
import { elementItemsController } from '../../controllers/markers/elementItems.js';

export const elementItemsRouter = Router();

elementItemsRouter.get('/', elementItemsController.getAll);
elementItemsRouter.get('/:id', elementItemsController.getById);
elementItemsRouter.post('/', elementItemsController.create);
elementItemsRouter.put('/:id', elementItemsController.update);
elementItemsRouter.delete('/:id', elementItemsController.delete);
