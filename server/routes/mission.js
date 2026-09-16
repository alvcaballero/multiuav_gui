import { Router } from 'express';

import { missionController } from '../controllers/mission.js';
export const createMissionRouter = () => {
  const missionRouter = Router();

  missionRouter.get('/', missionController.getMission);
  missionRouter.post('/', missionController.createMission);
  missionRouter.get('/routes', missionController.getRoutes);
  missionRouter.post('/sendTask', missionController.sendTask);
  missionRouter.post('/showXYZ', missionController.showMissionXYZ);

  // Manual mission execution: load creates Plan+Mission+Routes and loads each
  // drone; command promotes the loaded routes and starts the mission.
  missionRouter.post('/load', missionController.loadMissionManual);
  missionRouter.post('/command', missionController.commandMissionManual);

  // Coordinate conversion endpoints
  missionRouter.post('/convert/geodetic-to-xyz', missionController.convertGeodeticToXYZ);
  missionRouter.post('/convert/xyz-to-geodetic', missionController.convertXYZToGeodetic);

  // Mission plan persistence endpoints
  missionRouter.get('/plans', missionController.getMissionPlans);
  missionRouter.post('/plans', missionController.createMissionPlan);
  missionRouter.get('/plans/:id', missionController.getMissionPlanById);
  missionRouter.get('/plans/show/:id', missionController.showMissionPlan);

  // Collision detection endpoints
  missionRouter.post('/validate', missionController.validateCollisions);
  missionRouter.post('/resolve', missionController.resolveCollisions);

  return missionRouter;
};
