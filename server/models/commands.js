import { DevicesModel } from '../models/devices.js';
import { eventsModel } from '../models/events.js';
import { getDatetime, writeYAML } from '../common/utils.js';
import { rosController } from '../controllers/ros.js';

export class commandsModel {
  static getSaveCommands(deviceId) {
    let deviceid = deviceId;

    console.log('devices acction ' + deviceid);
    return [];
  }

  static getCommandTypes(deviceid) {
    let response = [
      { type: 'custom' },
      { type: 'ResumeMission' },
      { type: 'StopMission' },
      { type: 'Gimbal' },
      { type: 'GimbalPitch' },
      { type: 'ResetGimbal' },
      { type: 'SincroniseFiles' },
      { type: 'threat' },
      { type: 'setupcamera' },
      { type: 'configureMission' },
      { type: 'commandMission' },
    ];
    console.log('devices acction ' + deviceid);
    return response;
  }

  static async sendCommand({ deviceId, type, attributes }) {
    console.log('POST API command send');
    console.log({ deviceId, type, attributes });
    //here get id and description, where description is string like threat,1 or sincronize, landing,1
    let response = { state: 'info', msg: 'Command no found' };
    if (deviceId >= 0) {
      response = {
        state: 'error',
        msg: 'Command to:' + DevicesModel.get_device_ns(deviceId) + ' no exist',
      };
    }

    if (type == 'loadMission') {
      response = await this.loadmissionDevice(deviceId, attributes);
    }
    if (type == 'commandMission') {
      response = await this.commandMissionDevice(deviceId);
    }
    if (deviceId >= 0) {
      if (type == 'threat') {
        response = await this.standarCommand(deviceId, 'threat'); //threatUAV(deviceId);
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

      eventsModel.addEvent({
        type: response.state,
        eventTime: getDatetime(),
        deviceId: deviceId,
        attributes: { message: response.msg },
      });
    }

    console.log(response);
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
    console.log('standar comand ' + uav_id);
    let response = {};
    //ros
    if (attributes) {
      response = await rosController.callService({ uav_id, type, request: attributes });
    } else {
      response = await rosController.callService({ uav_id, type });
    }
    // other

    return response;
  }

  static async loadmissionDevice(deviceId, routes, callback = (x) => x) {
    console.log('load mission device ' + deviceId);

    let response = { state: 'warning', msg: 'UAV no asing mission' };
    if (Object.values(routes).length == 0) {
      response = { state: 'info', msg: 'no mission' };
      return response;
    }
    for (const route of routes) {
      let myDevice = DevicesModel.getByName(route.uav);
      if (myDevice && (deviceId < 0 || deviceId == myDevice.id)) {
        console.log('load mission to ' + myDevice.id);
        let attributes = await rosController.decodeMissionMsg({ uav_id: myDevice.id, route });
        if (attributes) {
          response = await this.standarCommand(myDevice.id, 'configureMission', attributes);
          callback(response);
        } else {
          response = { state: 'warning', msg: 'UAV no asing mission' };
        }
      } else {
        response = { state: 'warning', msg: 'device not found' };
      }
      if (deviceId < 0) {
        eventsModel.addEvent({
          type: response.state,
          eventTime: getDatetime(),
          deviceId: myDevice.id,
          attributes: { message: response.msg },
        });
      }
    }
    console.log('finish load mission');
    return response;
  }

  static async commandMissionDevice(deviceId, callback = (x) => x) {
    let r = true;
    let alldevices = await DevicesModel.getAll();
    let response = { state: 'error', msg: 'Mission canceled' };
    for (const device_id of Object.keys(alldevices)) {
      let finding = false;
      if (Array.isArray(deviceId)) {
        finding = deviceId.some((device) => device == DevicesModel.get_device_ns(device_id));
      }
      if (deviceId < 0 || deviceId == device_id || finding) {
        console.log('command mission to ' + device_id);

        response = await this.standarCommand(device_id, 'commandMission', { data: true });

        callback(response);
        if (deviceId < 0) {
          eventsModel.addEvent({
            type: response.state,
            eventTime: getDatetime(),
            deviceId: device_id,
            attributes: { message: response.msg },
          });
        }
      }
    }
    return response;
  }
}
