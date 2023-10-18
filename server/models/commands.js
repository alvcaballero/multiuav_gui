export class commandsModel {
  static getCommandTypes({ id }) {
    let deviceid = id;
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
    return res.json(response);
  }

  static async sendCommand(deviceId, type, attributes) {
    console.log('POST API command send');
    console.log(req.body);
    //here get id and description, where description is string like threat,1 or sincronize, landing,1
    let response = { state: 'info', msg: 'Command no found' };
    if (deviceId >= 0) {
      response = {
        state: 'error',
        msg: 'Command to:' + data.get_device_ns(deviceId) + ' no exist',
      };
    }

    if (type == 'loadMission') {
      response = await loadmissionDevice(deviceId, attributes);
    }
    if (type == 'commandMission') {
      response = await commandMissionDevice(deviceId);
    }
    if (deviceId >= 0) {
      if (type == 'threat') {
        response = await standarCommand(deviceId, 'threat'); //threatUAV(deviceId);
      }
      if (type == 'SincroniseFiles') {
        response = await standarCommand(deviceId, 'sincronize');
      }
      if (type == 'ResumeMission') {
        response = await standarCommand(deviceId, 'resumemission');
      }
      if (type == 'StopMission') {
        response = await standarCommand(deviceId, 'stopMission');
      }
      if (type == 'Pausemission') {
        response = await standarCommand(deviceId, 'pausemission');
      }
      if (type == 'Gimbal') {
        response = await GimbalUAV(deviceId, attributes);
      }
      if (type == 'GimbalPitch') {
        response = await GimbalUAV(deviceId, attributes);
      }
      if (type == 'ResetGimbal') {
        response = await GimbalUAV(deviceId, { reset: true });
      }
      if (type == 'setupcamera') {
        response = await standarCommand(deviceId, 'setupcamera', attributes);
      }

      data.addEvent({
        type: response.state,
        eventTime: getDatetime(),
        deviceId: deviceId,
        attributes: { message: response.msg },
      });
    }

    console.log(response);
    return res.json(response);
  }

  static async GimbalUAV(uav_id, attributes) {
    let statuscommand = await standarCommand(uav_id, 'Gimbal', {
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
    let uavname = data.get_device_ns(uav_id);
    let uavcategory = data.get_device_category(uav_id);
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
    let alldevices = data.state.devices;
    let response = { state: 'warning', msg: 'UAV no asing mission' };
    if (Object.values(mission).length == 0) {
      response = { state: 'info', msg: 'no mission' };
    }
    Object.keys(alldevices).forEach(async (device_id) => {
      if (deviceId < 0 || deviceId == device_id) {
        console.log('load mission to ' + device_id);
        let uavcategory = data.get_device_category(device_id);
        if (devices_msg[uavcategory]['services'].hasOwnProperty('configureMission')) {
          let attributes = loadMission(device_id, mission);
          if (attributes) {
            response = await standarCommand(device_id, 'configureMission', attributes);
          } else {
            response = { state: 'warning', msg: 'UAV no asing mission' };
          }
        } else {
          response = { state: 'warning', msg: 'UAV service load mission' };
        }
        if (deviceId < 0) {
          data.addEvent({
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
    let alldevices = data.state.devices;
    let response = { state: 'error', msg: 'Mission canceled' };
    Object.keys(alldevices).forEach(async (device_id) => {
      if (deviceId < 0 || deviceId == device_id) {
        let uavcategory = data.get_device_category(device_id);
        console.log('command mission to ' + device_id);
        if (uavcategory !== 'dji_M300') {
          response = await standarCommand(device_id, 'commandMission', { data: true });
        } else {
          response = await standarCommand(device_id, 'commandMission');
        }
        if (deviceId < 0) {
          data.addEvent({
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
}
