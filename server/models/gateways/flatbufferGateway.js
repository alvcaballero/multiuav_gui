import { getFlatbufferServer } from '../flatbuffer/index.js';
import { logger } from '../../common/logger.js';

export function createFlatbufferGateway({ getServer = getFlatbufferServer } = {}) {
  return {
    protocol: 'robofleet',

    // fbEncode codifica por key (configureMission, commandMission), no por msgType: recibe la
    // key del endpoint y el mensaje del command encoder tal cual.
    async sendCommand({ device, endpoint, message }) {
      logger.debug(`sending via robofleet device deviceId=${device.id}`);
      const server = getServer();
      if (!server) return { state: 'error', msg: 'FlatBuffer disabled in server configuration' };
      return server.sendCommand({ uav_id: device.id, type: endpoint.key, attributes: message });
    },
  };
}
