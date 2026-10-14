import { MqttServer } from './MqttServer.js';
export { MqttServer };

// Shared instance — initialized by server.js, consumed by commands.js
let _instance = null;

export function initMqttServer(url, options) {
  _instance = new MqttServer(url, options);
  return _instance;
}

export function getMqttServer() {
  return _instance;
}
