import { Router } from 'express';
import { markersController } from '../controllers/markers/markers.js';
import {
  elementTypesController,
  upload,
  setIconAssetType,
  setModelAssetType,
} from '../controllers/markers/elementTypes.js';
import { elementGroupsController } from '../controllers/markers/elementGroups.js';
import { elementItemsController } from '../controllers/markers/elementItems.js';
import { basesController } from '../controllers/markers/bases.js';
import { assignmentsController } from '../controllers/markers/assignments.js';

// All /api/markers/* routes live here so the full surface can be reviewed in
// one place instead of hunting across per-resource files that all share the
// same base path. Routes are grouped by resource prefix (/types, /groups,
// /items, /bases, /assignments) directly on this single markersRouter.
export const markersRouter = Router();

// ─── /api/markers/types — element type catalog ─────────────────────────────
markersRouter.get('/types/', elementTypesController.getAll);
markersRouter.get('/types/:id', elementTypesController.getById);
markersRouter.post('/types/', elementTypesController.create);
markersRouter.put('/types/:id', elementTypesController.update);
markersRouter.delete('/types/:id', elementTypesController.delete);

// Assets por tipo (icono 2D y modelo 3D)
markersRouter.get('/types/:id/icon', elementTypesController.serveIcon);
markersRouter.get('/types/:id/model', elementTypesController.serveModel);
markersRouter.post('/types/:id/icon', setIconAssetType, upload.single('file'), elementTypesController.uploadIcon);
markersRouter.post('/types/:id/model', setModelAssetType, upload.single('file'), elementTypesController.uploadModel);

// ─── /api/markers/groups — element groups ──────────────────────────────────
markersRouter.get('/groups/', elementGroupsController.getAll);
markersRouter.get('/groups/:id', elementGroupsController.getById);
markersRouter.get('/groups/:id/items', elementGroupsController.getItems);
markersRouter.post('/groups/', elementGroupsController.create);
markersRouter.put('/groups/:id', elementGroupsController.update);
markersRouter.delete('/groups/:id', elementGroupsController.delete);

// ─── /api/markers/items — element items ────────────────────────────────────
markersRouter.get('/items/', elementItemsController.getAll);
markersRouter.get('/items/:id', elementItemsController.getById);
markersRouter.get('/items/:id/detail', elementItemsController.getDetailById);
markersRouter.post('/items/', elementItemsController.create);
markersRouter.put('/items/:id', elementItemsController.update);
markersRouter.delete('/items/:id', elementItemsController.delete);

// ─── /api/markers/bases — inspection bases ─────────────────────────────────
markersRouter.get('/bases/', basesController.getAll);
markersRouter.get('/bases/assigned', basesController.getAssigned);
markersRouter.get('/bases/:id', basesController.getById);
markersRouter.post('/bases/', basesController.create);
markersRouter.put('/bases/:id', basesController.update);
markersRouter.delete('/bases/:id', basesController.delete);

// ─── /api/markers/assignments — base/element assignments ──────────────────
markersRouter.get('/assignments/', assignmentsController.getAll);
markersRouter.get('/assignments/settings', assignmentsController.getSettings);
markersRouter.get('/assignments/:id', assignmentsController.getById);
markersRouter.post('/assignments/', assignmentsController.create);
markersRouter.put('/assignments/:id', assignmentsController.update);
markersRouter.delete('/assignments/:id', assignmentsController.delete);

// ─── /api/markers — root instances ({markersbase, elements}) ──────────────
// Consumed by get_registered_objects in mcp_server. Mounted last so it never
// shadows the more specific prefixes above.
markersRouter.get('/', markersController.getMarkers);
markersRouter.post('/', markersController.setMarkers);
markersRouter.get('/bases', markersController.getBases);
markersRouter.get('/bases/assigned', markersController.getBasesWithAssignments);
markersRouter.get('/elements', markersController.getElements);
