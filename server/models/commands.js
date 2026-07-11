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
import logger from '../common/logger.js';

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
    // validateDevicesMsgKeys al cargar).
    const available = [...Object.keys(categoryConfig?.services ?? {}), ...Object.keys(categoryConfig?.actions ?? {})];
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
      Boolean(categoryConfig?.services?.[command.requires] ?? categoryConfig?.actions?.[command.requires]);
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
      attributes: { message: response.msg },
    });

    logger.debug(`sendCommand response: ${JSON.stringify(response)}`);
    return response;
  }

  static async GimbalUAV(uav_id, attributes) {
    let statuscommand = await this.standarCommand(uav_id, 'Gimbal', {
      header: { seq: 0, stamp: { secs: 0, nsecs: 0 }, frame_id: '' },
      is_reset: attributes.reset ? true : false,
      payload_index: 0,
      rotationMode: 0, // rotation cooradiration 0 = execute angle command based on the previously set reference point,1 = execute angle command based on the current point
      pitch: attributes.pitch ? attributes.pitch : 0.0,
      roll: attributes.roll ? attributes.roll : 0.0,
      yaw: attributes.yaw ? attributes.yaw : 0.0,
      time: 0.0,
    });
    return statuscommand;
  }
  static async standarCommand(uav_id, type, attributes) {
    logger.debug(`standarCommand uavId=${uav_id} type=${type}`);
    let response = {};
    //ros
    const myDevice = await devicesController.getDevice(uav_id);
    if (myDevice.protocol == 'ros') {
      // Mismo `type` puede vivir en services: o actions: según la categoría del
      // device (ej. CameraFileDownload es service en unas y action en otras) —
      // services: gana si aparece en ambos bloques.
      const categoryConfig = categoryModel.getCategory(myDevice.category);
      const isAction = !categoryConfig?.services?.hasOwnProperty(type) && categoryConfig?.actions?.hasOwnProperty(type);
      try {
        if (isAction) {
          logger.debug(`sending via ROS action uavId=${uav_id}`);
          response = await rosController.sendActionGoal({ uav_id, type, message: attributes ?? {} });
        } else {
          logger.debug(`sending via ROS device uavId=${uav_id}`);
          response = await rosController.callService({ uav_id, type, request: attributes });
        }
      } catch (error) {
        const errMsg = error instanceof Error ? error.message : String(error);
        logger.error(`standarCommand ROS error uavId=${uav_id} type=${type}: ${errMsg}`);
        response = { state: 'error', msg: errMsg };
      }
    }
    //robofleet
    if (myDevice.protocol == 'robofleet') {
      logger.debug(`sending via robofleet device uavId=${uav_id}`);

      response = await getFlatbufferServer().sendCommand({ uav_id, type, attributes });
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
   * Loads a mission to a SINGLE device: receives the FULL mission and internally
   * extracts the route that belongs to `deviceId` (matched by UAV name), then
   * sends it via the ROS `configureMission` service.
   * Fleet fan-out + plan/mission/route persistence live in missionModel (manual)
   * or initMission + missionExecutionSM (automatic).
   * @param {number} deviceId
   * @param {object} missionData - full mission ({ route: [...], version })
   */
  static async loadMissionToDevice(deviceId, missionData) {
    logger.info(`loadMissionToDevice deviceId=${deviceId}`);
    const routes = missionData?.route ?? missionData;
    if (!Array.isArray(routes) || routes.length === 0) {
      return { state: 'info', msg: 'no mission' };
    }
    const myDevice = await devicesController.getDevice(deviceId);
    if (!myDevice) {
      return { state: 'warning', msg: `device ${deviceId} not found` };
    }
    const route = routes.find((r) => r.uav === myDevice.name);
    if (!route) {
      return { state: 'warning', msg: `device ${myDevice.name} not found in mission route` };
    }
    if (!route.wp || Object.values(route.wp).length === 0) {
      return { state: 'warning', msg: `route for ${route.uav} has no waypoints` };
    }
    const rawRoute = { ...route, uav_type: myDevice.category };
    return await this.standarCommand(deviceId, 'configureMission', rawRoute);
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
