import { Router } from 'express';
import { markersController } from '../../controllers/markers/markers.js';

export const markersRouter = Router();

// ─── Marker instances ─────────────────────────────────────────────────────────
// The element type catalog (/types*) lives under /api/markers/types via
// elementTypesRouter (routes/markers/elementTypes.js) — must NOT be
// duplicated here.
markersRouter.get('/', markersController.getMarkers);
markersRouter.post('/', markersController.setMarkers);
markersRouter.get('/bases', markersController.getBases);
markersRouter.get('/bases/assigned', markersController.getBasesWithAssignments);
markersRouter.get('/elements', markersController.getElements);
