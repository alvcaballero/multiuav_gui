import { Router } from 'express';

import { mapController } from '../controllers/map.js';

export const mapRouter = Router();

mapRouter.get('/elevation', mapController.getElevation);
mapRouter.post('/elevation', mapController.calcElevation);
