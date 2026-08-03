import { Router } from 'express';
import { assignmentsController } from '../../controllers/markers/assignments.js';

export const assignmentsRouter = Router();

assignmentsRouter.get('/', assignmentsController.getAll);
assignmentsRouter.get('/settings', assignmentsController.getSettings);
assignmentsRouter.get('/:id', assignmentsController.getById);
assignmentsRouter.post('/', assignmentsController.create);
assignmentsRouter.put('/:id', assignmentsController.update);
assignmentsRouter.delete('/:id', assignmentsController.delete);
