import { MavlinkServer } from './MavlinkServer.js';
export { MavlinkServer };

// Shared instance — initialized by server.js, consumed by the mavlink gateway
let _instance = null;

export function initMavlinkServer(links, options) {
  _instance = new MavlinkServer(links, options);
  return _instance;
}

export function getMavlinkServer() {
  return _instance;
}
