import { getMqttServer } from '../mqtt/index.js';
import { logger } from '../../common/logger.js';

export function createMqttGateway({ getServer = getMqttServer } = {}) {
  return {
    protocol: 'mqtt',

    async sendCommand({ device, type, attributes }) {
      logger.debug(`sending via mqtt device deviceId=${device.id}`);
      const server = getServer();
      if (!server) return { state: 'error', msg: 'MQTT disabled in server configuration' };
      return server.sendCommand({ uav_id: device.id, type, attributes });
    },
  };
}
