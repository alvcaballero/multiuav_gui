import { Router } from 'express';

import { serverController } from '../controllers/server.js';

export const serverRouter = Router();

serverRouter.get('/', serverController.server);
serverRouter.get('/datetime', serverController.getDateTime);
serverRouter.get('/protocol', serverController.getServerProtocol);
