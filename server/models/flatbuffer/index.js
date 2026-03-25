export { encodeFbMsg } from './fbEncode.js';
export { decodeFbMsg, decodeServiceResponse, isServiceResponse, getNameFromTopic } from './fbDecode.js';

import { FlatbufferServer } from './FlatbufferServer.js';
export { FlatbufferServer };

// Shared instance — initialized by server.js, consumed by commands.js
let _instance = null;

export function initFlatbufferServer(port) {
  _instance = new FlatbufferServer(port);
  return _instance;
}

export function getFlatbufferServer() {
  return _instance;
}
