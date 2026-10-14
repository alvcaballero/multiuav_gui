import { rosController } from '../../controllers/ros.js';
import { categoryModel } from '../category.js';
import { logger } from '../../common/logger.js';

// The same command `type` can live under services:, actions: or publishers: depending on
// the category (Gimbal is a service on the OSDK and a publisher on the PSDK; CameraFileDownload
// is a service on some and an action on others). Priority: services > actions > publishers.
//
// controllers/ros.js reaches commands.js (which loads this module) through its own imports, so
// `rosController` is still in its temporal dead zone when the gateway is registered: it is
// resolved when a command is sent, never as a default evaluated at registration time.
export function createRosGateway({ controller, categories } = {}) {
  return {
    protocol: 'ros',

    async sendCommand({ device, type, attributes }) {
      const ctl = controller ?? rosController;
      const catalog = categories ?? categoryModel;
      const deviceId = device.id;
      const categoryConfig = catalog.getCategory(device.category);
      const hasService = categoryConfig?.services?.hasOwnProperty(type);
      const hasAction = categoryConfig?.actions?.hasOwnProperty(type);
      const isAction = !hasService && hasAction;
      const isPublisher = !hasService && !hasAction && categoryConfig?.publishers?.hasOwnProperty(type);
      try {
        if (isPublisher) {
          logger.debug(`sending via ROS publisher deviceId=${deviceId}`);
          return await ctl.publishTopicDevice({ deviceId, type, message: attributes ?? {} });
        }
        if (isAction) {
          logger.debug(`sending via ROS action deviceId=${deviceId}`);
          return await ctl.sendActionGoalDevice({ deviceId, type, message: attributes ?? {} });
        }
        logger.debug(`sending via ROS device deviceId=${deviceId}`);
        return await ctl.callServiceDevice({ deviceId, type, request: attributes });
      } catch (error) {
        const errMsg = error instanceof Error ? error.message : String(error);
        logger.error(`ROS gateway error deviceId=${deviceId} type=${type}: ${errMsg}`);
        return { state: 'error', msg: errMsg };
      }
    },
  };
}
