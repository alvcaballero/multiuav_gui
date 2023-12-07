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
    console.log(type + ' uav_id' + uav_id);
    let uavname = DevicesModel.get_device_ns(uav_id);
    let uavcategory = DevicesModel.get_device_category(uav_id);
    console.log(type + ' --' + uavname + '--' + uavcategory);

    if (!devices_msg[uavcategory]['services'].hasOwnProperty(type)) {
      console.log(type + ' to:' + uavname + ' dont have this service');
      return { state: 'error', msg: type + ' to:' + uavname + ' dont have this service' };
    }

    let standarmessage = new ROSLIB.Service({
      ros: ros,
      name: uavname + devices_msg[uavcategory]['services'][type]['name'],
      serviceType: devices_msg[uavcategory]['services'][type]['serviceType'],
    });
    let request;
    if (attributes) {
      request = new ROSLIB.ServiceRequest(attributes);
    } else {
      request = new ROSLIB.ServiceRequest({});
    }

    return new Promise((resolve, rejects) => {
      standarmessage.callService(
        request,
        function (result) {
          console.log('send command  ');
          console.log(result);
          if (result.success || result.result) {
            resolve({
              state: 'success',
              msg: type + ' to' + uavname + ' ok',
            });
          } else {
            resolve({
              state: 'error',
              msg: type + ' to:' + uavname + ' error',
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

  static async loadmissionDevice(deviceId, mission) {
    console.log('load mission device ' + deviceId);
    let alldevices = await DevicesModel.getAll();
    let response = { state: 'warning', msg: 'UAV no asing mission' };
    if (Object.values(mission).length == 0) {
      response = { state: 'info', msg: 'no mission' };
    }
    Object.keys(alldevices).forEach(async (device_id) => {
      if (deviceId < 0 || deviceId == device_id) {
        console.log('load mission to ' + device_id);
        let uavcategory = DevicesModel.get_device_category(device_id);
        if (devices_msg[uavcategory]['services'].hasOwnProperty('configureMission')) {
          let attributes = this.loadMission(device_id, mission);
          if (attributes) {
            response = await this.standarCommand(device_id, 'configureMission', attributes);
          } else {
            response = { state: 'warning', msg: 'UAV no asing mission' };
          }
        } else {
          response = { state: 'warning', msg: 'UAV service load mission' };
        }
        if (deviceId < 0) {
          eventsModel.addEvent({
            type: response.state,
            eventTime: getDatetime(),
            deviceId: device_id,
            attributes: { message: response.msg },
          });
        }
      }
    });
    return response;
  }

  static async commandMissionDevice(deviceId) {
    let r = true;
    let alldevices = await DevicesModel.getAll();
    let response = { state: 'error', msg: 'Mission canceled' };
    Object.keys(alldevices).forEach(async (device_id) => {
      if (deviceId < 0 || deviceId == device_id) {
        let uavcategory = DevicesModel.get_device_category(device_id);
        console.log('command mission to ' + device_id);
        if (true) {
          //uavcategory !== 'dji_M300'
          response = await this.standarCommand(device_id, 'commandMission', { data: true });
        } else {
          response = await this.standarCommand(device_id, 'commandMission');
        }
        if (deviceId < 0) {
          eventsModel.addEvent({
            type: response.state,
            eventTime: getDatetime(),
            deviceId: device_id,
            attributes: { message: response.msg },
          });
        }
      }
    });
    return response;
  }

  static loadMission(uav_id, mission) {
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

    Object.values(mission).forEach((route) => {
      if (route['uav'] == uavname) {
        console.log('route');
        console.log(route);
        idle_vel = route.attributes.hasOwnProperty('idle_vel')
          ? route.attributes['idle_vel']
          : idle_vel;
        max_vel = route.attributes.hasOwnProperty('max_vel')
          ? route.attributes['max_vel']
          : max_vel;
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
    });
    return response;
  }
}
