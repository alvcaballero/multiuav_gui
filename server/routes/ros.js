import { Router } from 'express';

import { rosController } from '../controllers/ros.js';

export const rosRouter = Router();

rosRouter.get('/topics', rosController.getTopics);
rosRouter.get('/topics_type', rosController.getTopicType);
rosRouter.get('/message_details', rosController.getMessageDetails);
rosRouter.get('/publisher', rosController.getPublishers);
rosRouter.get('/services', rosController.getServices);
rosRouter.post('/publish', rosController.pubTopicOnce);
rosRouter.get('/subscribe_once', rosController.subscribeOnce);
