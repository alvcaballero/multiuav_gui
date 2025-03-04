import { Router } from 'express';

import { missionController } from '../controllers/mission.js';
export const createMissionRouter = () => {
  const missionRouter = Router();

  missionRouter.get('/', missionController.getMission); //get current mission//missionState
  missionRouter.get('/routes', missionController.getRoutes); //get current mission//missionState
  missionRouter.post('/sendTask', missionController.sendTask);
  return missionRouter;
};
