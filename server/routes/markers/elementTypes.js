import { Router } from 'express';
import { elementTypesController, upload } from '../../controllers/markers/elementTypes.js';

export const elementTypesRouter = Router();

elementTypesRouter.get('/', elementTypesController.getAll);
elementTypesRouter.get('/:id', elementTypesController.getById);
elementTypesRouter.post('/', elementTypesController.create);
elementTypesRouter.put('/:id', elementTypesController.update);
elementTypesRouter.delete('/:id', elementTypesController.delete);

// Assets por tipo (icono 2D y modelo 3D)
elementTypesRouter.get('/:id/icon', elementTypesController.serveIcon);
elementTypesRouter.get('/:id/model', elementTypesController.serveModel);

elementTypesRouter.post(
  '/:id/icon',
  (req, res, next) => {
    req.assetType = 'icon';
    next();
  },
  upload.single('file'),
  elementTypesController.uploadIcon
);

elementTypesRouter.post(
  '/:id/model',
  (req, res, next) => {
    req.assetType = 'model';
    next();
  },
  upload.single('file'),
  elementTypesController.uploadModel
);
