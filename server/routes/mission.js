import { Router } from 'express';

import { CreateController } from '../controllers/mission.js';

export const createMissionRouter = ({ model }) => {
  const missionRouter = Router();
  const missionController = CreateController({ model });

  missionRouter.get('/', missionController.getMission); //get current mission//missionState
  missionRouter.get('/routes', missionController.getRoutes); //get current mission//missionState
  missionRouter.post('/sendTask', missionController.sendTask);
  //missionRouter.get('/updateFiles/:id_uav/:id_mission', missionController.updateFiles); //Download files form UAV
  //missionRouter.get('/showFiles/:id_uav/:id_mission', missionController.showFiles); //list files UAV
  //missionRouter.get('/listFiles/:id_mission/:id_uav', missionController.listFiles); // list of file in server
  //[missionRouter.post('/setMission', missionController.setMission);

  return missionRouter;
};
