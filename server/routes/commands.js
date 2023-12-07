import { Router } from 'express';

import { commandsController } from '../controllers/commands.js';

export const commandsRouter = Router();

commandsRouter.get('/types', commandsController.getAvalaibleCommands);
commandsRouter.get('/send', commandsController.getSaveCommands);
commandsRouter.post('/send', commandsController.sendCommand);
