import { devicesController } from '../../controllers/devices.js';
import { eventsController } from '../../controllers/events.js';
import { getGateway } from '../gateways/index.js';
import { positionsController } from '../../controllers/positions.js';
import { categoryModel } from '../category.js';
import { deviceModelsModel } from '../deviceModels/deviceModels.js';
import { CommandType } from '../../config/commandTypes.js';
import { logger } from '../../common/logger.js';
import { commandSchema, resolveCommand, validateAttributes } from './commandSchema.js';
import { commandRegistry, missionTaskForDevice } from './commandRegistry.js';
import { formatProblem, taskProblems } from '../mission/missionValidator.js';
import { contentsOf } from './commandEncoders.js';
import { encodeMsg, hasMsgEncoder } from '../msgEncode.js';

const COMMAND_TYPES = new Set(Object.values(CommandType));

// Boot: el msgType de cada endpoint que implementa un command tiene encoder en msgEncode (o un
// passthrough explícito); si no, saldría crudo al UAV sin aviso. Vive acá y no en category.js
// porque msgEncode → encoders de misión → category: allá cerraría un ciclo de imports.
// Misma política que category.js: warn, o throw con LOG_LEVEL=debug.
function validateMsgEncoders() {
  const problems = categoryModel.getAll().flatMap((category) =>
    [...commandRegistry.resolveCommands(categoryModel.getCategory(category)).commands]
      .filter(([, endpoint]) => endpoint && !hasMsgEncoder(endpoint.msgType))
      .map(([type, endpoint]) => `${category}.${type}: msgType '${endpoint.msgType}' has no encoder (msgEncode.js)`)
  );
  if (problems.length === 0) return;
  const msg = `msgEncode: ${problems.length} problem(s): ${problems.join('; ')}`;
  if (process.env.LOG_LEVEL === 'debug') throw new Error(msg);
  logger.warn(msg);
}

validateMsgEncoders();

// endpoint resuelto → cómo se lo muestra (/types) y se lo loguea: `publishers.VehicleCommand`.
const viaOf = (endpoint) => (endpoint ? `${endpoint.block}.${endpoint.key}` : null);

export class commandsModel {
  static getSaveCommands(deviceId) {
    let deviceid = deviceId;
    logger.debug(`getSaveCommands deviceId=${deviceid}`);
    return [];
  }

  // Commands que expone el device: los deriva commandRegistry de los endpoints que su
  // categoría declara en devices_msg.yaml (más los locales), y cada uno sale resuelto contra
  // command_schema + overrides de la categoría. `via` dice qué endpoint lo implementa;
  // `requires` (su key, null si es local) se mantiene por compatibilidad.
  // Only the commands of the capabilities the device HAS (deviceModelsModel.allowsCommand): its
  // interface may implement more (ardupilot_mavlink has Takeoff; griffin on it is telemetry only).
  static async getCommandTypes(deviceId) {
    logger.debug(`getCommandTypes deviceId=${deviceId}`);
    const device = await devicesController.getDevice(deviceId);
    const categoryConfig = device?.category ? categoryModel.getCategory(device.category) : undefined;
    const { commands } = commandRegistry.resolveCommands(categoryConfig);
    return [...commands]
      .filter(([type]) => !device || deviceModelsModel.allowsCommand(device, type))
      .map(([type, endpoint]) => ({
      ...resolveCommand(commandSchema, type, categoryConfig),
      type,
      requires: endpoint?.key ?? null,
      via: viaOf(endpoint),
    }));
  }

  // 1. device + type  2. ¿la categoría lo implementa? (commandRegistry)  3. attributes contra
  // command_schema  4. el encoder del endpoint arma el mensaje  5. el gateway del protocolo lo
  // manda por ESE endpoint. Un command local (SaveHome) corre en el server, sin endpoint.
  // LoadMission/CommandMission son POR-DEVICE: el fan-out de flota + plan/mission/tasks vive
  // en missionModel (flujo manual) o initMission + missionExecutionSM (flujo automático).
  static async sendCommand({ deviceId, type, attributes }) {
    logger.info(`sendCommand deviceId=${deviceId} type=${type}`);
    logger.debug(`sendCommand attributes: ${JSON.stringify(attributes)}`);
    if (!(deviceId >= 0)) {
      return { state: 'info', msg: 'Command no found' };
    }

    const device = await devicesController.getDevice(deviceId);
    if (!device) {
      return { state: 'error', msg: `device ${deviceId} not found` };
    }

    if (!COMMAND_TYPES.has(type)) {
      return { state: 'error', msg: `Command type '${type}' does not exist` };
    }

    const categoryConfig = categoryModel.getCategory(device.category);
    const { commands } = commandRegistry.resolveCommands(categoryConfig);
    if (!commands.has(type)) {
      return { state: 'error', msg: `Command '${type}' not supported by device ${device.name}` };
    }
    // The interface implements it, but the vehicle may not do it (its device model's capabilities).
    if (!deviceModelsModel.allowsCommand(device, type)) {
      return {
        state: 'error',
        msg: `Command '${type}' needs a capability device ${device.name} (model ${device.deviceModel}) does not have`,
      };
    }

    // Contrato del command resuelto para ESTE device (el mismo que entregó /types): los
    // encoders reciben valores ya validados y con defaults. freeform = payload validado por
    // otro módulo (LoadMission → taskGraph).
    const resolved = resolveCommand(commandSchema, type, categoryConfig);
    if (resolved && !resolved.freeform) {
      const checked = validateAttributes(resolved, attributes);
      if (!checked.ok) return { state: 'error', msg: checked.msg };
      attributes = checked.values;
    }

    const endpoint = commands.get(type);
    const response = endpoint
      ? await this.sendToDevice(device, type, endpoint, attributes)
      : commandRegistry.localRun(type)({ deviceId, attributes, positions: positionsController });

    eventsController.addEvent({
      type: response.state,
      deviceId: deviceId,
      attributes: { action: type, message: response.msg },
    });

    logger.debug(`sendCommand response: ${JSON.stringify(response)}`);
    return response;
  }

  // Encoder del endpoint → mensaje → gateway del protocolo del device. Un encoder que
  // rechaza (ej. Takeoff sin posición conocida, misión sin task para el device) es un error.
  static async sendToDevice(device, type, endpoint, attributes) {
    // A device without a stored protocol is a ROS one: same default as DeviceSchema and devices.create.
    const protocol = device.protocol ?? 'ros';
    const gateway = getGateway(protocol);
    if (!gateway) {
      return { state: 'error', msg: `No gateway for protocol '${protocol}' (device ${device.name})` };
    }
    // Takeoff/MoveRelative/Gimbal parten de dónde está el UAV ahora.
    const position = await positionsController.getByDeviceId(device.id);
    // What the vehicle IS (its device model): a VTOL mission item on anything else is refused.
    // Resolved here, not in commandRegistry: deviceModels imports it (through the capabilities).
    const vehicleType = deviceModelsModel.platformOf(device.deviceModel)?.vehicleType ?? null;
    // Every load reaches the device through here, the MCP's bare LoadMission too: the mission
    // validator's per-task rules (what this device can carry) refuse it before anything is sent.
    if (type === CommandType.LOAD_MISSION) {
      try {
        const errors = taskProblems(missionTaskForDevice(attributes, device, vehicleType), device);
        if (errors.length > 0) {
          return { state: 'error', msg: `Mission refused: ${errors.map(formatProblem).join('; ')}` };
        }
      } catch (err) {
        return { state: 'error', msg: err.message };
      }
    }
    // ① qué se manda (commandRegistry, dominio) → ② la forma que define el msgType del endpoint
    // (msgEncode, la misma para todo protocolo) → ③ el gateway solo empaqueta.
    let messages;
    try {
      const content = commandRegistry.encode(type, endpoint, { attributes, position, device, vehicleType });
      messages = contentsOf(content).map((c) => encodeMsg(endpoint.msgType, c));
    } catch (err) {
      return { state: 'error', msg: err.message };
    }
    logger.debug(`sendToDevice ${type} via ${viaOf(endpoint)} deviceId=${device.id} (${messages.length} msg)`);
    // Una secuencia va en orden y corta en el primer fallo: mover sin haber tomado el control no sirve.
    let response;
    for (const message of messages) {
      response = await gateway.sendCommand({ device, endpoint, message });
      if (response?.state !== 'success') return response;
    }
    return response;
  }
}
