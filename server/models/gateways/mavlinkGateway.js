import { getMavlinkServer } from '../mavlink/index.js';
import { logger } from '../../common/logger.js';

export function createMavlinkGateway({ getServer = getMavlinkServer } = {}) {
  return {
    protocol: 'mavlink',

    async sendCommand({ device, endpoint, message }) {
      logger.debug(`sending via mavlink device deviceId=${device.id}`);
      const server = getServer();
      if (!server) return { state: 'error', msg: 'MAVLink disabled in server configuration' };
      // The endpoint's msgType is the dialect: the server checks it against the vehicle's heartbeat.
      return server.sendCommand({ device, endpoint, message });
    },
  };
}
