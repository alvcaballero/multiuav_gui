import { Router } from 'express';

import { planningController } from '../controllers/planning.js';

export const planningRouter = Router();

planningRouter.get('/mission', planningController.getMissionTypes);
planningRouter.get('/missionstype', planningController.getTypes);
planningRouter.get('/missionparam/:type', planningController.getParam);
planningRouter.get('/getDefault', planningController.getDefault);
planningRouter.post('/setDefault', planningController.setDefault);
planningRouter.post('/setMarkers', planningController.setMarkers);
planningRouter.get('/getBases', planningController.getBases);
planningRouter.get('/getMarkers', planningController.getElements);
