import { devicesController } from '../controllers/devices.js';
import { eventsController } from '../controllers/events.js';
import { getGateway } from './gateways/index.js';
import { positionsController } from '../controllers/positions.js';
import { categoryModel } from './category.js';
import {
  CommandType,
  DEFAULT_COMMAND_TYPES,
  typesForServiceKey,
  commandDef,
  Dispatch,
  Payload,
  KNOWN_COMMAND_TYPES,
} from '../config/commandCatalog.js';
import { logger } from '../common/logger.js';
import { normalizeMission } from './mission/taskGraph.js';
import { buildVehicleCommand } from './vehicleCommand.js';
import { commandSchema, resolveCommand, validateAttributes } from './commandSchema.js';
import { PublisherKey } from '../config/deviceMsgCatalog.js';

export class commandsModel {
  static getSaveCommands(deviceId) {
    let deviceid = deviceId;
    logger.debug(`getSaveCommands deviceId=${deviceid}`);
    return [];
  }

  static async getCommandTypes(deviceId) {
    logger.debug(`getCommandTypes deviceId=${deviceId}`);
    const types = [...DEFAULT_COMMAND_TYPES];

    const category = (await devicesController.getDevice(deviceId))?.category;
    const categoryConfig = category ? categoryModel.getCategory(category) : undefined;

    // Capacidades ROS de la categoría (keys de devices_msg.yaml). Cada command
    // cuyo `requires` matchea una capacidad se expone; typesForServiceKey hace el
    // lookup inverso. Una key sin command asociado devuelve [] (ya avisada por
    // validateDevicesMsgKeys al cargar). Se incluyen publishers: un command
    // (ej. Gimbal) puede satisfacerse por service, action o publisher.
    const available = [
      ...Object.keys(categoryConfig?.services ?? {}),
      ...Object.keys(categoryConfig?.actions ?? {}),
      ...Object.keys(categoryConfig?.publishers ?? {}),
    ];
    for (const serviceKey of available) {
      types.push(...typesForServiceKey(serviceKey));
    }

    // Cada command sale resuelto para esta categoría (command_schema + overrides de
    // devices_msg): params con unidades/rangos/defaults, label, confirm… `requires` se
    // mantiene. Set: una key declarada en dos bloques no duplica sus commands.
    return [...new Set(types)].map((type) => ({
      ...resolveCommand(commandSchema, type, categoryConfig),
      type,
      requires: commandDef(type)?.requires ?? null,
    }));
  }

  static async sendCommand({ deviceId, type, attributes }) {
    logger.info(`sendCommand deviceId=${deviceId} type=${type}`);
    logger.debug(`sendCommand attributes: ${JSON.stringify(attributes)}`);
    //here get id and description, where description is string like threat,1 or sincronize, landing,1
    if (!(deviceId >= 0)) {
      return { state: 'info', msg: 'Command no found' };
    }

    const myDevice = await devicesController.getDevice(deviceId);
    if (!myDevice) {
      return { state: 'error', msg: `device ${deviceId} not found` };
    }

    if (!KNOWN_COMMAND_TYPES.has(type)) {
      return { state: 'error', msg: `Command type '${type}' does not exist` };
    }

    const command = commandDef(type);
    const categoryConfig = categoryModel.getCategory(myDevice.category);
    const available =
      command.requires == null ||
      Boolean(
        categoryConfig?.services?.[command.requires] ??
        categoryConfig?.actions?.[command.requires] ??
        categoryConfig?.publishers?.[command.requires]
      );
    if (!available) {
      return { state: 'error', msg: `Command '${type}' not supported by device ${myDevice.name}` };
    }

    // Contrato del command resuelto para ESTE device (el mismo que entregó /types): los
    // handlers reciben valores ya validados y con defaults. freeform = payload libre o
    // validado por otro módulo (LoadMission → taskGraph).
    const resolved = resolveCommand(commandSchema, type, categoryConfig);
    if (resolved && !resolved.freeform) {
      const checked = validateAttributes(resolved, attributes);
      if (!checked.ok) return { state: 'error', msg: checked.msg };
      attributes = checked.values;
    }

    let response;
    // loadMission/commandMission son POR-DEVICE: cargan/comandan a UN dron. El
    // fan-out de flota + creación de plan/mission/routes vive en missionModel
    // (flujo manual) o initMission + missionExecutionSM (flujo automático).
    if (type == CommandType.LOAD_MISSION) {
      response = await this.loadMissionToDevice(deviceId, attributes);
    } else if (type == CommandType.COMMAND_MISSION) {
      response = await this.commandMissionToDevice(deviceId);
    } else if (command.dispatch === Dispatch.LOCAL) {
      // saveHome: efecto local en el server, sin ROS.
      positionsController.updatePosition({ deviceId, setHome: true });
      response = { state: 'success', msg: 'Home saved' };
    } else if (command.dispatch === Dispatch.GIMBAL) {
      // Gimbal enruta por GimbalUAV; ResetGimbal fuerza el flag de reset.
      const gimbalAttrs = command.payload === Payload.RESET ? { reset: true } : attributes;
      response = await this.GimbalUAV(deviceId, gimbalAttrs);
    } else if (command.dispatch === Dispatch.VEHICLE_COMMAND) {
      response = await this.vehicleCommand(deviceId, type, attributes);
    } else if (command.dispatch === Dispatch.SERVICE) {
      // ROS service/action.
      const request = command.payload === Payload.ATTRIBUTES ? attributes : undefined;
      response = await this.standarCommand(deviceId, command.rosService, request);
    } else {
      response = { state: 'error', msg: `Command '${type}' has no dispatch handler` };
    }

    eventsController.addEvent({
      type: response.state,
      deviceId: deviceId,
      attributes: { action: type, message: response.msg },
    });

    logger.debug(`sendCommand response: ${JSON.stringify(response)}`);
    return response;
  }

  static async GimbalUAV(deviceId, attributes) {
    // Objeto de dominio NEUTRO: ángulos ABSOLUTOS en grados. El wire format ROS lo arma
    // rosEncode.js según el msgType de la categoría: dji_osdk_ros/GimbalAction (OSDK, service)
    // o psdk_interfaces/msg/GimbalRotation (PSDK, publisher). standarCommand elige
    // el transporte (service vs publisher) según lo que declara el devices_msg.
    // Los dos wire formats llevan siempre los 3 ejes: el eje que no viene se completa con
    // el ángulo actual (attributes.gimbal = [pitch, roll, yaw]), así mandar solo {pitch}
    // no mueve roll/yaw. Sin telemetría de gimbal la posición trae [0, 0, 0].
    // Los ejes que sí vienen ya están validados por sendCommand (command_schema).
    const position = await positionsController.getByDeviceId(deviceId);
    const [pitch, roll, yaw] = position?.attributes?.gimbal ?? [0, 0, 0];
    return await this.standarCommand(deviceId, 'Gimbal', {
      reset: Boolean(attributes.reset),
      pitch: attributes.pitch ?? pitch,
      roll: attributes.roll ?? roll,
      yaw: attributes.yaw ?? yaw,
    });
  }
  static async vehicleCommand(deviceId, type, attributes) {
    // Takeoff/moveRelative turn into absolute PX4 targets, so they need where the UAV is now.
    const position = await positionsController.getByDeviceId(deviceId);
    let msg;
    try {
      msg = buildVehicleCommand({ type, attributes, position });
    } catch (err) {
      return { state: 'error', msg: err.message };
    }
    return await this.standarCommand(deviceId, PublisherKey.VEHICLE_COMMAND, msg);
  }
  static async standarCommand(deviceId, type, attributes) {
    logger.debug(`standarCommand deviceId=${deviceId} type=${type}`);
    const device = await devicesController.getDevice(deviceId);
    if (!device) return { state: 'error', msg: `device ${deviceId} not found` };
    // A device without a stored protocol is a ROS one: same default as DeviceSchema and devices.create.
    const protocol = device.protocol ?? 'ros';
    const gateway = getGateway(protocol);
    if (!gateway) {
      return { state: 'error', msg: `No gateway for protocol '${protocol}' (device ${device.name})` };
    }
    return gateway.sendCommand({ device, type, attributes });
  }

  /**
   * Loads a mission to a SINGLE device via the ROS `configureMission` service.
   * Accepts any mission shape normalizeMission does (tasks[], route[], or a bare
   * route array — what the MCP load_mission_to_uav tool posts) and loads the one
   * task that belongs to `deviceId`. The mission flows pass missionForTask(), so
   * that task is always unambiguous; a mission with several tasks for this device
   * is rejected rather than guessing which one to fly.
   * Fleet fan-out + plan/mission/task persistence live in missionModel.
   * @param {number} deviceId
   * @param {object|object[]} missionData
   */
  static async loadMissionToDevice(deviceId, missionData) {
    logger.info(`loadMissionToDevice deviceId=${deviceId}`);
    if (missionData == null || (Array.isArray(missionData) && missionData.length === 0)) {
      return { state: 'info', msg: 'no mission' };
    }
    const myDevice = await devicesController.getDevice(deviceId);
    if (!myDevice) {
      return { state: 'warning', msg: `device ${deviceId} not found` };
    }

    let mission;
    try {
      mission = normalizeMission(Array.isArray(missionData) ? { route: missionData } : missionData);
    } catch (err) {
      return { state: 'error', msg: err.message };
    }
    const deviceTasks = mission.tasks.filter((t) => t.device === myDevice.name);
    if (deviceTasks.length === 0) {
      return { state: 'warning', msg: `device ${myDevice.name} has no task in this mission` };
    }
    if (deviceTasks.length > 1) {
      const ids = deviceTasks.map((t) => t.task_id).join(', ');
      return { state: 'error', msg: `device ${myDevice.name} has several tasks (${ids}); load one task at a time` };
    }
    const task = { ...deviceTasks[0], uav_type: myDevice.category };
    return await this.standarCommand(deviceId, 'configureMission', task);
  }

  /**
   * Commands (starts) the loaded mission on a SINGLE device.
   * @param {number} deviceId
   */
  static async commandMissionToDevice(deviceId) {
    logger.info(`commandMissionToDevice deviceId=${deviceId}`);
    return await this.standarCommand(deviceId, 'commandMission', { data: true });
  }
}
