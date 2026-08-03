import { Router } from 'express';
import { elementGroupsController } from '../../controllers/markers/elementGroups.js';

export const elementGroupsRouter = Router();

elementGroupsRouter.get('/', elementGroupsController.getAll);
elementGroupsRouter.get('/:id', elementGroupsController.getById);
elementGroupsRouter.get('/:id/items', elementGroupsController.getItems);
elementGroupsRouter.post('/', elementGroupsController.create);
elementGroupsRouter.put('/:id', elementGroupsController.update);
elementGroupsRouter.delete('/:id', elementGroupsController.delete);
