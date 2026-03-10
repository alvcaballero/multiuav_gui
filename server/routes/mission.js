import { Router } from 'express';

import { missionController } from '../controllers/mission.js';
export const createMissionRouter = () => {
  const missionRouter = Router();

  missionRouter.get('/', missionController.getMission); //get current mission//missionState
  missionRouter.post('/', missionController.createMission); //set new mission//missionState
  missionRouter.get('/routes', missionController.getRoutes); //get current mission//missionState
  missionRouter.post('/sendTask', missionController.sendTask);
  missionRouter.post('/showXYZ', missionController.showMissionXYZ);

  // Mission plan persistence endpoints
  missionRouter.get('/plans', missionController.getMissionPlans);         // GET /missions/plans
  missionRouter.get('/plans/:id', missionController.getMissionPlanById);  // GET /missions/plans/:id
  missionRouter.get('/plans/show/:id', missionController.showMissionPlan);        // POST /missions/plans
  // Collision detection endpoints
  missionRouter.post('/validate', missionController.validateCollisions); // Validate mission for collisions
  missionRouter.post('/resolve', missionController.resolveCollisions); // Validate and auto-resolve collisions

  return missionRouter;
};
