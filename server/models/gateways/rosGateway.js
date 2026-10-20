import { rosController } from '../../controllers/ros.js';
import { logger } from '../../common/logger.js';

// The endpoint arrives already resolved (commandEncoders: which block/key of the category
// implements the command), so the transport is read off `endpoint.block`, never guessed.
//
// controllers/ros.js reaches commands.js (which loads this module) through its own imports, so
// `rosController` is still in its temporal dead zone when the gateway is registered: it is
// resolved when a command is sent, never as a default evaluated at registration time.
export function createRosGateway({ controller } = {}) {
  return {
    protocol: 'ros',

    async sendCommand({ device, endpoint, message }) {
      const ctl = controller ?? rosController;
      const deviceId = device.id;
      const { block, key: type } = endpoint;
      try {
        if (block === 'publishers') {
          logger.debug(`sending via ROS publisher deviceId=${deviceId}`);
          return await ctl.publishTopicDevice({ deviceId, type, message: message ?? {} });
        }
        if (block === 'actions') {
          logger.debug(`sending via ROS action deviceId=${deviceId}`);
          return await ctl.sendActionGoalDevice({ deviceId, type, message: message ?? {} });
        }
        logger.debug(`sending via ROS service deviceId=${deviceId}`);
        return await ctl.callServiceDevice({ deviceId, type, request: message });
      } catch (error) {
        const errMsg = error instanceof Error ? error.message : String(error);
        logger.error(`ROS gateway error deviceId=${deviceId} type=${type}: ${errMsg}`);
        return { state: 'error', msg: errMsg };
      }
    },
  };
}
