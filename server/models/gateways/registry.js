// Outbound port of the device layer: one gateway per device protocol.
//
//   DeviceGateway {
//     protocol: string                                   // value of device.protocol
//     sendCommand({ device, endpoint, message }) -> Promise<{ state, msg }>
//   }
//
// `endpoint` = { block, key, msgType } of the category, already resolved by commandEncoders
// (block: services | actions | publishers); `message` = what the command encoder built.
//
// Callers (commandsModel) pick the gateway by `device.protocol` and know nothing about
// ROS, FlatBuffers or MQTT. Adding a protocol means registering a gateway here, not
// adding another `if (protocol == ...)` to the callers.

const gateways = new Map();

export function registerGateway(gateway) {
  if (!gateway?.protocol || typeof gateway.sendCommand !== 'function') {
    throw new Error('registerGateway: a gateway needs a `protocol` and a `sendCommand` function');
  }
  if (gateways.has(gateway.protocol)) {
    throw new Error(`registerGateway: protocol '${gateway.protocol}' is already registered`);
  }
  gateways.set(gateway.protocol, gateway);
  return gateway;
}

export const getGateway = (protocol) => gateways.get(protocol);

export const registeredProtocols = () => [...gateways.keys()];
