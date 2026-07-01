import { devicesController } from '../controllers/devices.js';
import { eventsController } from '../controllers/events.js';
import { rosController } from '../controllers/ros.js';
import { getFlatbufferServer } from './flatbuffer/index.js';
import { positionsController } from '../controllers/positions.js';
import { missionWpTracking } from './mission/missionWpTracking.js';
import { categoryModel } from './category.js';
import { DEFAULT_COMMAND_TYPES, typesForServiceKey, commandDef, Dispatch, Payload } from '../config/commandCatalog.js';
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
    let response = { state: 'info', msg: 'Command no found' };
    if (deviceId >= 0) {
      response = {
        state: 'error',
        msg: 'Command to:' + devicesController.getDevice(deviceId)?.name + ' no exist',
      };
    }

    if (type == 'loadMission') {
      response = await this.loadmissionDevice(deviceId, attributes);
      if (deviceId < 0) {
        eventsController.addEvent({
          type: response.state,
          deviceId: null,
          attributes: { message: response.msg },
        });
      }
    }
    if (type == 'commandMission') {
      response = await this.commandMissionDevice(deviceId);
      if (deviceId < 0) {
        eventsController.addEvent({
          type: response.state,
          deviceId: null,
          attributes: { message: response.msg },
        });
      }
    }
    if (deviceId >= 0) {
      // Despacho por command def. Los FLEET (loadMission/commandMission) ya se
      // ejecutaron arriba (soportan deviceId<0); acá se despacha el resto según
      // su `dispatch`. Un type sin def deja el 'Command no found' inicial.
      const command = commandDef(type);
      if (command && command.dispatch !== Dispatch.FLEET) {
        if (command.dispatch === Dispatch.LOCAL) {
          // saveHome: efecto local en el server, sin ROS.
          positionsController.updatePosition({ deviceId, setHome: true });
          response = { state: 'success', msg: 'Home saved' };
        } else if (command.dispatch === Dispatch.GIMBAL) {
          // Gimbal enruta por GimbalUAV; ResetGimbal fuerza el flag de reset.
          const gimbalAttrs = command.payload === Payload.RESET ? { reset: true } : attributes;
          response = await this.GimbalUAV(deviceId, gimbalAttrs);
        } else if (command.dispatch === Dispatch.SERVICE) {
          // ROS service. rosService undefined (custom) → standarCommand lo maneja.
          const request = command.payload === Payload.ATTRIBUTES ? attributes : undefined;
          response = await this.standarCommand(deviceId, command.rosService, request);
        }
      }

      eventsController.addEvent({
        type: response.state,
        deviceId: deviceId,
        attributes: { message: response.msg },
      });
    }

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
    let myDevice = await devicesController.getDevice(uav_id);
    if (myDevice.protocol == 'ros') {
      logger.debug(`sending via ROS device uavId=${uav_id}`);
      try {
        if (attributes) {
          response = await rosController.callService({ uav_id, type, request: attributes });
        } else {
          response = await rosController.callService({ uav_id, type });
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

  static async loadmissionDevice(deviceId, routes, callback = (x) => x) {
    logger.info(`loadmissionDevice deviceId=${deviceId}`);

    let response = { state: 'warning', msg: 'UAV no asing mission' };
    if (Object.values(routes).length == 0) {
      response = { state: 'info', msg: 'no mission' };
      return response;
    }

    const loadedDeviceIds = [];

    for (const route of routes) {
      logger.debug(`load route for uav ${route.uav}`);
      let myDevice = await devicesController.getByName(route.uav);
      logger.debug(`device found in route: id=${myDevice?.id} name=${myDevice?.name} searched=${deviceId}`);
      if (myDevice && (deviceId < 0 || deviceId == myDevice.id)) {
        logger.info(`loading mission to device ${myDevice.id}`);
        if (!route.wp || Object.values(route.wp).length === 0) {
          response = { state: 'warning', msg: `route for ${route.uav} has no waypoints` };
        } else {
          const rawRoute = { ...route, uav_type: myDevice.category };
          response = await this.standarCommand(myDevice.id, 'configureMission', rawRoute);
          callback(response);
          if (response.state !== 'error') loadedDeviceIds.push(myDevice.id);
          if (deviceId >= 0) break;
        }
      } else {
        response = { state: 'warning', msg: `device ${route.uav} not found in mission route` };
      }
      if (deviceId < 0) {
        eventsController.addEvent({
          type: response.state,
          deviceId: myDevice ? myDevice.id : null,
          attributes: { message: response.msg },
        });
      }
    }

    if (loadedDeviceIds.length > 0) {
      const missionData = { route: routes, version: '3' };
      missionWpTracking
        .onMissionLoaded(loadedDeviceIds, missionData)
        .catch((err) => logger.error(`WpTracking onMissionLoaded error: ${err.message}`));
    }

    logger.info('finish load mission');
    return response;
  }

  static async commandMissionDevice(deviceId, callback = (x) => x) {
    let alldevices = await devicesController.getAllDevices();
    let response = { state: 'error', msg: 'Mission canceled' };
    const commandedDeviceIds = [];

    for (const device of alldevices) {
      let finding = Array.isArray(deviceId) && deviceId.some((id) => id == device.id);
      if (deviceId < 0 || deviceId == device.id || finding) {
        logger.info(`commandMissionDevice sending to device ${device.id}`);
        response = await this.standarCommand(device.id, 'commandMission', { data: true });
        callback(response);
        if (response.state !== 'error') commandedDeviceIds.push(device.id);
        if (deviceId < 0) {
          eventsController.addEvent({
            type: response.state,
            deviceId: device.id,
            attributes: { message: response.msg },
          });
        }
      }
    }

    if (commandedDeviceIds.length > 0) {
      missionWpTracking
        .onMissionCommanded(commandedDeviceIds)
        .catch((err) => logger.error(`WpTracking onMissionCommanded error: ${err.message}`));
    }

    return response;
  }
}
