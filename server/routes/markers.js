import { Router } from 'express';
import { markersController, upload } from '../controllers/markers.js';

export const markersRouter = Router();

// ─── Marker instances ─────────────────────────────────────────────────────────
markersRouter.get('/', markersController.getMarkers);
markersRouter.post('/', markersController.setMarkers);
markersRouter.get('/bases', markersController.getBases);
markersRouter.get('/bases/assigned', markersController.getBasesWithAssignments);
markersRouter.get('/elements', markersController.getElements);

// ─── Element type catalog ─────────────────────────────────────────────────────
markersRouter.get('/types', markersController.getAllTypes);
markersRouter.post('/types', markersController.createCustomType);
markersRouter.delete('/types/:id', markersController.deleteCustomType);

// Assets por tipo (icono 2D y modelo 3D)
markersRouter.get('/types/:id/icon', markersController.serveIcon);
markersRouter.get('/types/:id/model', markersController.serveModel);

markersRouter.post('/types/:id/icon', (req, res, next) => {
  req.assetType = 'icon';
  next();
}, upload.single('file'), markersController.uploadIcon);

markersRouter.post('/types/:id/model', (req, res, next) => {
  req.assetType = 'model';
  next();
}, upload.single('file'), markersController.uploadModel);
