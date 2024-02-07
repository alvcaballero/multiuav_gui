import { Router } from 'express';

import { missionController } from '../controllers/mission.js';

export const missionRouter = Router();

missionRouter.get('/', missionController.getmission); //get current mission//missionState
missionRouter.post('/sendTask', missionController.sendTask);
missionRouter.post('/setMission', missionController.setMission);
missionRouter.get('/updateFiles/:id_uav/:id_mission', missionController.updateFiles); //Download files form UAV
missionRouter.get('/showFiles/:id_uav/:id_mission', missionController.showFiles); //list files UAV
missionRouter.get('/listFiles/:id_mission/:id_uav', missionController.listFiles); // list of file in server
