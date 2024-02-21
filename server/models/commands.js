import { ros } from '../models/ros.js';
import { DevicesModel } from '../models/devices.js';
import { eventsModel } from '../models/events.js';
import { readYAML, getDatetime } from '../common/utils.js';
import ROSLIB from 'roslib';

const devices_msg = readYAML('../config/devices/devices_msg.yaml');

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
  static standarCommand(uav_id, type, attributes) {
    console.log('StandarCommand - ' + type + '- uav_id - ' + uav_id);
    let uavName = DevicesModel.get_device_ns(uav_id);
    let uavCategory = DevicesModel.get_device_category(uav_id);
    console.log(type + ' --' + uavName + '--' + uavCategory);

    if (!devices_msg[uavCategory]['services'].hasOwnProperty(type)) {
      console.log(type + ' to:' + uavName + ' dont have this service');
      return { state: 'error', msg: type + ' to:' + uavName + ' dont have this service' };
    }

    let standarMessage = new ROSLIB.Service({
      ros: ros,
      name: uavName + devices_msg[uavCategory]['services'][type]['name'],
      serviceType: devices_msg[uavCategory]['services'][type]['serviceType'],
    });
    let request;
    if (attributes) {
      request = new ROSLIB.ServiceRequest(attributes);
    } else {
      request = new ROSLIB.ServiceRequest({});
    }

    return new Promise((resolve, rejects) => {
      standarMessage.callService(
        request,
        function (result) {
          console.log('send command  ');
          console.log(result);
          if (result.success || result.result) {
            resolve({
              state: 'success',
              msg: type + ' to' + uavName + ' ok',
            });
          } else {
            resolve({
              state: 'error',
              msg: type + ' to:' + uavName + ' error',
            });
          }
        },
        function (result) {
          console.log('Error:' + result);
          resolve({ state: 'error', msg: 'Error:' + result });
        }
      );
    });
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
        if (devices_msg[myDevice.category]['services'].hasOwnProperty('configureMission')) {
          let attributes = this.PrepareMissionMsg(myDevice.id, route);
          if (attributes) {
            response = await this.standarCommand(myDevice.id, 'configureMission', attributes);
            callback(response);
          } else {
            response = { state: 'warning', msg: 'UAV no asing mission' };
          }
        } else {
          response = { state: 'warning', msg: 'UAV service load mission' };
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

  static PrepareMissionMsg(uav_id, route) {
    console.log('  load  - mission' + uav_id);
    let uavname = DevicesModel.get_device_ns(uav_id);
    let uavcategory = DevicesModel.get_device_category(uav_id);
    let mode_yaw = 0;
    let mode_gimbal = 0;
    let mode_trace = 0;
    let idle_vel = 1.8;
    let max_vel = 10;
    let mode_landing = 0;
    let response = null;
    let wp_command = [];
    let yaw_pos = [];
    let speed_pos = [];
    let gimbal_pos = [];
    let action_matrix = [];
    let param_matrix = [];

    if (route['uav'] == uavname) {
      //console.log('route');
      //console.log(route);
      idle_vel = route.attributes.hasOwnProperty('idle_vel')
        ? route.attributes['idle_vel']
        : idle_vel;
      max_vel = route.attributes.hasOwnProperty('max_vel') ? route.attributes['max_vel'] : max_vel;
      mode_yaw = route.attributes.hasOwnProperty('mode_yaw')
        ? route.attributes['mode_yaw']
        : mode_yaw;
      mode_gimbal = route.attributes.hasOwnProperty('mode_gimbal')
        ? route.attributes['mode_gimbal']
        : mode_gimbal;
      mode_trace = route.attributes.hasOwnProperty('mode_trace')
        ? route.attributes['mode_trace']
        : mode_trace;
      mode_landing = route.attributes.hasOwnProperty('mode_landing')
        ? route.attributes['mode_landing']
        : mode_landing;

      Object.values(route['wp']).forEach((item) => {
        let yaw, gimbal, speed;
        let action_array = Array(10).fill(0);
        let param_array = Array(10).fill(0);
        let pos = new ROSLIB.Message({
          latitude: item.pos[0],
          longitude: item.pos[1],
          altitude: item.pos[2],
        });
        yaw = item.hasOwnProperty('yaw') ? item.yaw : 0;
        speed = item.hasOwnProperty('speed') ? item.speed : idle_vel;
        gimbal = item.hasOwnProperty('gimbal') ? item.gimbal : 0;
        if (item.hasOwnProperty('action')) {
          Object.keys(item.action).forEach((action_val, index, arr) => {
            let found = Object.values(
              devices_msg[uavcategory]['attributes']['mission_action']
            ).find((element) => element.name == action_val);
            if (found) {
              action_array[index] = Number(found.id);
              param_array[index] = found.param ? Number(item.action[action_val]) : 0;
            }
          });
        }
        wp_command.push(pos);
        gimbal_pos.push(gimbal);
        yaw_pos.push(yaw);
        speed_pos.push(speed);
        action_matrix.push(action_array);
        param_matrix.push(param_array);
      });

      let yaw_pos_msg = new ROSLIB.Message({ data: yaw_pos });
      let speed_pos_msg = new ROSLIB.Message({ data: speed_pos });
      let gimbal_pos_msg = new ROSLIB.Message({ data: gimbal_pos });
      let action_matrix_msg = new ROSLIB.Message({
        data: action_matrix.flat(),
      });
      let param_matrix_msg = new ROSLIB.Message({
        data: param_matrix.flat(),
      });

      response = {
        type: 'waypoint',
        waypoint: wp_command,
        radius: 0,
        maxVel: max_vel,
        idleVel: idle_vel,
        yaw: yaw_pos_msg,
        speed: speed_pos_msg,
        gimbalPitch: gimbal_pos_msg,
        yawMode: mode_yaw,
        traceMode: mode_trace,
        gimbalPitchMode: mode_gimbal,
        finishAction: mode_landing,
        commandList: action_matrix_msg,
        commandParameter: param_matrix_msg,
      };
    }

    return response;
  }
}
