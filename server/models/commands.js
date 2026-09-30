import { devicesController } from '../controllers/devices.js';
import { eventsController } from '../controllers/events.js';
import { rosController } from '../controllers/ros.js';
import { getFlatbufferServer } from './flatbuffer/index.js';
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

    return types.map((type) => ({ type }));
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
    } else if (command.dispatch === Dispatch.SERVICE) {
      // ROS service/action. rosService undefined (custom) → standarCommand lo maneja.
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
    // Objeto de dominio NEUTRO (grados). El wire format ROS lo arma rosEncode.js
    // según el msgType de la categoría: dji_osdk_ros/GimbalAction (OSDK, service)
    // o psdk_interfaces/msg/GimbalRotation (PSDK, publisher). standarCommand elige
    // el transporte (service vs publisher) según lo que declara el devices_msg.
    let statuscommand = await this.standarCommand(deviceId, 'Gimbal', {
      reset: attributes.reset ? true : false,
      pitch: attributes.pitch ? attributes.pitch : 0.0,
      roll: attributes.roll ? attributes.roll : 0.0,
      yaw: attributes.yaw ? attributes.yaw : 0.0,
    });
    return statuscommand;
  }
  static async standarCommand(deviceId, type, attributes) {
    logger.debug(`standarCommand deviceId=${deviceId} type=${type}`);
    let response = {};
    //ros
    const myDevice = await devicesController.getDevice(deviceId);
    if (myDevice.protocol == 'ros') {
      // Mismo `type` puede vivir en services:, actions: o publishers: según la
      // categoría del device (ej. Gimbal es service en las OSDK y publisher en el
      // PSDK; CameraFileDownload es service en unas y action en otras). Prioridad:
      // services > actions > publishers.
      const categoryConfig = categoryModel.getCategory(myDevice.category);
      const hasService = categoryConfig?.services?.hasOwnProperty(type);
      const hasAction = categoryConfig?.actions?.hasOwnProperty(type);
      const isAction = !hasService && hasAction;
      const isPublisher = !hasService && !hasAction && categoryConfig?.publishers?.hasOwnProperty(type);
      try {
        if (isPublisher) {
          logger.debug(`sending via ROS publisher deviceId=${deviceId}`);
          response = await rosController.publishTopicDevice({ deviceId, type, message: attributes ?? {} });
        } else if (isAction) {
          logger.debug(`sending via ROS action deviceId=${deviceId}`);
          response = await rosController.sendActionGoalDevice({ deviceId, type, message: attributes ?? {} });
        } else {
          logger.debug(`sending via ROS device deviceId=${deviceId}`);
          response = await rosController.callServiceDevice({ deviceId, type, request: attributes });
        }
      } catch (error) {
        const errMsg = error instanceof Error ? error.message : String(error);
        logger.error(`standarCommand ROS error deviceId=${deviceId} type=${type}: ${errMsg}`);
        response = { state: 'error', msg: errMsg };
      }
    }
    //robofleet
    if (myDevice.protocol == 'robofleet') {
      logger.debug(`sending via robofleet device deviceId=${deviceId}`);

      response = await getFlatbufferServer().sendCommand({ uav_id: deviceId, type, attributes });
      if (response == {}) {
        response = {
          state: 'success',
          msg: type + ' to websocket ok',
        };
      }
    }
    //mavlink
    // other

    return response;
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
