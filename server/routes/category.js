import { Router } from 'express';

import { categoryController } from '../controllers/category.js';

export const categoryRouter = Router();

categoryRouter.get('/messages/', categoryController.messagesTypes);
categoryRouter.get('/atributes/:type', categoryController.getAtributes);
categoryRouter.get('/atributesparam/:type/:param', categoryController.getAtributesParam);
categoryRouter.get('/actions/:type', categoryController.getActions);
categoryRouter.get('/:category', categoryController.getCategory);
categoryRouter.put('/:category', categoryController.updateCategory);
categoryRouter.post('/:category', categoryController.createCategory);
categoryRouter.delete('/:category', categoryController.deleteCategory);
categoryRouter.get('/', categoryController.getAll);
