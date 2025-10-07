import { Router } from 'express';

import { rosController } from '../controllers/ros.js';

export const rosRouter = Router();

rosRouter.get('/topics', rosController.getTopics);
rosRouter.get('/topics_type', rosController.getTopicType);
rosRouter.get('/message_details', rosController.getMessageDetails);
rosRouter.get('/publisher', rosController.getPublishers);
rosRouter.get('/services', rosController.getServices);
rosRouter.get('/service_type', rosController.getServicesType);
rosRouter.get('/service_request_details', rosController.getServiceRequestDetails);
rosRouter.get('/service_response_details', rosController.getServiceResponseDetails);
rosRouter.post('/service_call', rosController.callRosService);
rosRouter.post('/publish', rosController.pubTopicOnce);
rosRouter.get('/subscribe_once', rosController.subscribeOnce);
