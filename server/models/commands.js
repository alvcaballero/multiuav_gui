import { devicesController } from '../controllers/devices.js';
import { eventsController } from '../controllers/events.js';
import { getDatetime } from '../common/utils.js';
import { rosController } from '../controllers/ros.js';
import { getFlatbufferServer } from './flatbuffer/index.js';
import { positionsController } from '../controllers/positions.js';
import { decodeMissionMsg } from './MissionDecoder.js';
import logger from '../common/logger.js';

export class commandsModel {
  static getSaveCommands(deviceId) {
    let deviceid = deviceId;
    logger.debug(`getSaveCommands deviceId=${deviceid}`);
    return [];
  }

  static getCommandTypes(deviceid) {
    let response = [
      { type: 'custom' },
      { type: 'saveHome' },
      { type: 'ResumeMission' },
      { type: 'Pausemission' },
      { type: 'StopMission' },
      { type: 'Gimbal' },
      { type: 'GimbalPitch' },
      { type: 'ResetGimbal' },
      { type: 'SincroniseFiles' },
      { type: 'threat_confirmation' },
      { type: 'threat_defuse' },
      { type: 'setupcamera' },
      { type: 'configureMission' },
      { type: 'commandMission' },
    ];
    logger.debug(`getCommandTypes deviceId=${deviceid}`);
    return response;
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
    }
    if (type == 'commandMission') {
      response = await this.commandMissionDevice(deviceId);
    }
    if (deviceId >= 0) {
      if (type == 'saveHome') {
        positionsController.updatePosition({ deviceId, setHome: true });
        response = { state: 'success', msg: 'Home saved' };
      }
      if (type == 'threat_confirmation') {
        response = await this.standarCommand(deviceId, 'threat_confirmation'); //threatUAV(deviceId);
      }
      if (type == 'threat_defuse') {
        response = await this.standarCommand(deviceId, 'threat_defuse'); //threatUAV(deviceId);
      }
      if (type == 'SincroniseFiles') {
        response = await this.standarCommand(deviceId, 'sincronize');
      }
      if (type == 'ResumeMission') {
        response = await this.standarCommand(deviceId, 'resumemission');
      }
      if (type == 'StopMission') {
        response = await this.standarCommand(deviceId, 'stopMission');
      }
      if (type == 'Pausemission') {
        response = await this.standarCommand(deviceId, 'pausemission');
      }
      if (type == 'Gimbal') {
        response = await this.GimbalUAV(deviceId, attributes);
      }
      if (type == 'GimbalPitch') {
        response = await this.GimbalUAV(deviceId, attributes);
      }
      if (type == 'ResetGimbal') {
        response = await this.GimbalUAV(deviceId, { reset: true });
      }
      if (type == 'setupcamera') {
        response = await this.standarCommand(deviceId, 'setupcamera', attributes);
      }
      if (type == 'CameraFileDownload') {
        response = await this.standarCommand(deviceId, 'CameraFileDownload', attributes);
      }
      if (type == 'custom') {
        response = await this.standarCommand(deviceId, undefined, attributes);
      }

      eventsController.addEvent({
        type: response.state,
        eventTime: getDatetime(),
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
      if (attributes) {
        response = await rosController.callService({ uav_id, type, request: attributes });
      } else {
        response = await rosController.callService({ uav_id, type });
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
    for (const route of routes) {
      logger.debug(`load route for uav ${route.uav}`);
      let myDevice = await devicesController.getByName(route.uav);
      logger.debug(`device found in route: id=${myDevice.id} name=${myDevice.name} searched=${deviceId}`);
      if (myDevice && (deviceId < 0 || deviceId == myDevice.id)) {
        logger.info(`loading mission to device ${myDevice.id}`);
        let attributes = await decodeMissionMsg({ uav_id: myDevice.id, route });
        if (attributes) {
          response = await this.standarCommand(myDevice.id, 'configureMission', attributes);
          callback(response);
          if (deviceId >=0){
            break;
          }
        } else {
          response = { state: 'warning', msg: 'UAV no asing mission' };
        }
      } else {
        response = { state: 'warning', msg: `device ${route.uav} not found in mission route` };
      }
      if (deviceId < 0) {
        eventsController.addEvent({
          type: response.state,
          eventTime: getDatetime(),
          deviceId: myDevice ? myDevice.id : null,
          attributes: { message: response.msg },
        });
      }
    }
    logger.info('finish load mission');
    return response;
  }

  static async commandMissionDevice(deviceId, callback = (x) => x) {
    let alldevices = await devicesController.getAllDevices();
    let response = { state: 'error', msg: 'Mission canceled' };
    for (const device of alldevices) {
      let finding = false;
      if (Array.isArray(deviceId)) {
        finding = deviceId.some((mydeviceId) => mydeviceId == device.id);
      }
      if (deviceId < 0 || deviceId == device.id || finding) {
        logger.info(`commandMissionDevice sending to device ${device.id}`);

        response = await this.standarCommand(device.id, 'commandMission', { data: true });

        callback(response);
        if (deviceId < 0) {
          eventsController.addEvent({
            type: response.state,
            eventTime: getDatetime(),
            deviceId: device.id,
            attributes: { message: response.msg },
          });
        }
      }
    }
    return response;
  }
}
