import { getFlatbufferServer } from '../flatbuffer/index.js';
import { logger } from '../../common/logger.js';

export function createFlatbufferGateway({ getServer = getFlatbufferServer } = {}) {
  return {
    protocol: 'robofleet',

    async sendCommand({ device, type, attributes }) {
      logger.debug(`sending via robofleet device deviceId=${device.id}`);
      const server = getServer();
      if (!server) return { state: 'error', msg: 'FlatBuffer disabled in server configuration' };
      return server.sendCommand({ uav_id: device.id, type, attributes });
    },
  };
}
