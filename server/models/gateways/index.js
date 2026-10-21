import { registerGateway } from './registry.js';
import { createRosGateway } from './rosGateway.js';
import { createFlatbufferGateway } from './flatbufferGateway.js';
import { createMqttGateway } from './mqttGateway.js';
import { createMavlinkGateway } from './mavlinkGateway.js';

export { getGateway, registeredProtocols } from './registry.js';

registerGateway(createRosGateway());
registerGateway(createFlatbufferGateway());
registerGateway(createMqttGateway());
registerGateway(createMavlinkGateway());
