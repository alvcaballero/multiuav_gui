import { DevicesController } from '../controllers/devices.js';
import { eventsController } from '../controllers/events.js';
import { getDatetime } from '../common/utils.js';
import { rosController } from '../controllers/ros.js';
import { categoryController } from '../controllers/category.js';
import { sendCommandToClient } from '../WebsocketDevices.js';

async function decodeMissionMsg({ uav_id, route }) {
  let device = await DevicesController.getDevice(uav_id);
  console.log(device);
  let response = null;
  let uavname = device.name;
  let uavcategory = device.category;
  let mode_yaw = 0;
  let mode_gimbal = 0;
  let mode_trace = 0;
  let idle_vel = 1.8;
  let max_vel = 10;
  let mode_landing = 0;
  let wp_command = [];
  let yaw_pos = [];
  let speed_pos = [];
  let gimbal_pos = [];
  let action_matrix = [];
  let param_matrix = [];
  if (route['uav'] == uavname) {
    //console.log('route'); //console.log(route);
    idle_vel = route.attributes.hasOwnProperty('idle_vel') ? route.attributes['idle_vel'] : idle_vel;
    max_vel = route.attributes.hasOwnProperty('max_vel') ? route.attributes['max_vel'] : max_vel;
    mode_yaw = route.attributes.hasOwnProperty('mode_yaw') ? route.attributes['mode_yaw'] : mode_yaw;
    mode_gimbal = route.attributes.hasOwnProperty('mode_gimbal') ? route.attributes['mode_gimbal'] : mode_gimbal;
    mode_trace = route.attributes.hasOwnProperty('mode_trace') ? route.attributes['mode_trace'] : mode_trace;
    mode_landing = route.attributes.hasOwnProperty('mode_landing') ? route.attributes['mode_landing'] : mode_landing;

    let categoryModel = await categoryController.getActionsParam({ type: uavcategory });

    Object.values(route['wp']).forEach((item) => {
      let yaw, gimbal, speed;
      let action_array = Array(10).fill(0);
      let param_array = Array(10).fill(0);
      let pos = {
        latitude: item.pos[0],
        longitude: item.pos[1],
        altitude: item.pos[2],
      };
      yaw = item.hasOwnProperty('yaw') ? item.yaw : 0;
      speed = item.hasOwnProperty('speed') ? item.speed : idle_vel;
      gimbal = item.hasOwnProperty('gimbal') ? item.gimbal : 0;

      console.log(item.action ?? 'no action');

      if (item.hasOwnProperty('action')) {
        Object.keys(item.action).forEach((action_val, index, arr) => {
          let found = Object.values(categoryModel).find((element) => element.name == action_val);
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

    response = {
      type: 'waypoint',
      waypoint: wp_command,
      radius: 0,
      maxVel: max_vel,
      idleVel: idle_vel,
      yaw: yaw_pos,
      speed: speed_pos,
      gimbalPitch: gimbal_pos,
      yawMode: mode_yaw,
      traceMode: mode_trace,
      gimbalPitchMode: mode_gimbal,
      finishAction: mode_landing,
      commandList: action_matrix,
      commandParameter: param_matrix,
    };
  }
  return response;
}

export class commandsModel {
  static getSaveCommands(deviceId) {
    let deviceid = deviceId;
    console.log('devices acction  save commands' + deviceid);
    return [];
  }

  static getCommandTypes(deviceid) {
    let response = [
      { type: 'custom' },
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
    console.log('devices acction get types ' + deviceid);
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
        msg: 'Command to:' + DevicesController.getDevice(deviceId).name + ' no exist',
      };
    }

    if (type == 'loadMission') {
      response = await this.loadmissionDevice(deviceId, attributes);
    }
    if (type == 'commandMission') {
      response = await this.commandMissionDevice(deviceId);
    }
    if (deviceId >= 0) {
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
    let myDevice = await DevicesController.getDevice(uav_id);
    console.log(myDevice);
    if (myDevice.protocol == 'ros') {
      console.log('ros device ros');
      if (attributes) {
        response = await rosController.callService({ uav_id, type, request: attributes });
      } else {
        response = await rosController.callService({ uav_id, type });
      }
    }
    //robofleet
    if (myDevice.protocol == 'robofleet') {
      console.log('robotflet device ros');

      if (attributes) {
        response = await sendCommandToClient({ uav_id, type, attributes });
      } else {
        response = await sendCommandToClient({ uav_id, type });
      }
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
    console.log('load mission device ' + deviceId);

    let response = { state: 'warning', msg: 'UAV no asing mission' };
    if (Object.values(routes).length == 0) {
      response = { state: 'info', msg: 'no mission' };
      return response;
    }
    for (const route of routes) {
      console.log('load route' + route.uav);
      let myDevice = await DevicesController.getByName(route.uav);
      if (myDevice && (deviceId < 0 || deviceId == myDevice.id)) {
        console.log('load mission to ' + myDevice.id);
        let attributes = await decodeMissionMsg({ uav_id: myDevice.id, route });
        if (attributes) {
          response = await this.standarCommand(myDevice.id, 'configureMission', attributes);
          callback(response);
        } else {
          response = { state: 'warning', msg: 'UAV no asing mission' };
        }
      } else {
        response = { state: 'warning', msg: `device ${route.uav} not found` };
      }
      if (deviceId < 0) {
        eventsController.addEvent({
          type: response.state,
          eventTime: getDatetime(),
          deviceId: myDevice ? myDevice.id : -1,
          attributes: { message: response.msg },
        });
      }
    }
    console.log('finish load mission');
    return response;
  }

  static async commandMissionDevice(deviceId, callback = (x) => x) {
    let r = true;
    let alldevices = await DevicesController.getAllDevices();
    let response = { state: 'error', msg: 'Mission canceled' };
    for (const device_id of Object.keys(alldevices)) {
      let finding = false;
      if (Array.isArray(deviceId)) {
        finding = deviceId.some((mydeviceId) => mydeviceId == device_id);
      }
      if (deviceId < 0 || deviceId == device_id || finding) {
        console.log('command mission to ' + device_id);

        response = await this.standarCommand(device_id, 'commandMission', { data: true });

        callback(response);
        if (deviceId < 0) {
          eventsController.addEvent({
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
