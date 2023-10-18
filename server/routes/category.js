import { Router } from 'express';

import { categoryController } from '../controllers/category.js';

export const categoryRouter = Router();

categoryRouter.get('/', categoryController.getall);
categoryRouter.get('/atributes/:type', categoryController.getAtributes);
categoryRouter.get('/atributesparam/:type/:param', categoryController.getAtributesParam);
categoryRouter.get('/actions/:type', categoryController.getActions);
