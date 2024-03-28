import { readJSON, readYAML, getDatetime } from '../common/utils.js';
import { StreamServer } from '../config/config.js';
import { eventsModel } from '../models/events.js';
import { rosController } from '../controllers/ros.js';
const devices_init = readYAML('../config/devices/devices_init.yaml');
const devices = {};
const devicesAccess = {};

const CheckDeviceOnline = setInterval(() => {
  let currentTime = new Date();
  let checkdevices = Object.keys(devices);
  checkdevices.forEach((element) => {
    //console.log(data.state.devices[element])
    //console.log(currentTime)
    if (currentTime - devices[element]['lastUpdate'] < 5000) {
      devices[element]['status'] = 'online';
    } else {
      devices[element]['status'] = 'offline';
    }
  });
}, 5000);

export class DevicesModel {
  constructor() {
    // conect with ros and other things
    console.log('constructor device model');
  }

  static async getAll(query) {
    if (query) {
      console.log(query);
      if (Array.isArray(query)) {
        const asArray = Object.entries(devices);
        const filtered = asArray.filter(([key, value]) => query.some((element) => key == element));
        return Object.fromEntries(filtered);
      }
      if (!isNaN(query)) {
        return devices[query] ? { id: devices[query] } : {};
      }
    }

    return devices;
  }
  static getAccess(id) {
    return Object.values(devicesAccess).find((device) => device.id === id);
  }

  static async create(device) {
    console.log(device);
    let serverState = rosController.getServerStatus();

    let uav_ns = device.name;
    let uav_type = device.category;
    let cur_uav_idx = String(Object.values(devices).length);

    console.log('create UAV');
    console.log('name' + uav_ns + '  type' + uav_type);

    let repeat_device = false;
    if (Object.values(devices).length > 0) {
      Object.values(devices).forEach((element) => {
        if (element) {
          repeat_device = element.name == uav_ns ? true : false;
        }
      });
    }

    if (repeat_device == true) {
      console.log('Dispositivo ya se encuentra registrado ' + uav_ns);
      return {
        state: 'error',
        msg: `Dispositivo ya se encuentra registrado ${uav_ns}`,
      };
    }

    this.updatedevice({
      id: cur_uav_idx,
      name: device.name,
      category: device.category,
      ip: device.ip,
      camera: device.camera,
      status: 'online',
      lastUpdate: null,
    });

    if (device.hasOwnProperty('user') && device.hasOwnProperty('pwd')) {
      this.updateDeviceAccess({
        id: cur_uav_idx,
        name: device.name,
        user: device.user,
        pwd: device.pwd,
        ip: device.ip,
      });
    }
    if (StreamServer) {
      this.addCameraWebRTC(device);
    }

    if (serverState.state === 'connect') {
      console.log('suscribe devices ');
      await rosController.subscribeDevice({
        id: cur_uav_idx,
        name: uav_ns,
        type: uav_type,
        camera: device.camera,
        watch_bound: true,
        bag: false,
      });

      console.log(devices);
      console.log('success', device.name + ' added. Type: ' + device.category);
      return { state: 'success', msg: 'conectado Correctamente' };
    } else {
      console.log('\nRos no está conectado.\n\n Por favor conéctelo primero.');
      return { state: 'error', msg: 'Ros no está conectado' };
    }
  }

  static async addCameraWebRTC(device) {
    for (let i = 0; i < device.camera.length; i = i + 1) {
      if (device.camera[i]['type'] == 'WebRTC') {
        await fetch(
          `http://localhost:9997/v3/config/paths/add/${device.name}_${device.camera[i].source}`,
          {
            method: 'POST',
            body: JSON.stringify({
              source: `rtsp://${device.ip}:8554/${device.camera[i].source}`,
            }),
            headers: {
              'Content-Type': 'application/json',
            },
          }
        );
      }
    }
  }

  static async getById({ id }) {
    return Object.values(devices).find((device) => device.id === id);
  }
  static getByName(name) {
    return Object.values(devices).find((device) => device.name === name);
  }

  static async delete({ id }) {
    console.log('remove id' + id);
    await this.removeDeviceCameraWebRTC(id);
    await this.removedevice({ id: id });

    let response = await rosController.unsubscribeDevice(id);
    return response;
  }

  static async removeDeviceCameraWebRTC(id) {
    let device_del = devices[id];
    if (device_del.hasOwnProperty('camera')) {
      for (let i = 0; i < device_del.camera.length; i = i + 1) {
        if (device_del.camera[i]['type'] == 'WebRTC') {
          await fetch(
            `http://localhost:9997/v3/config/paths/remove/${device_del.name}_${device_del.camera[i].source}`,
            {
              method: 'POST',
            }
          );
        }
      }
    }
  }

  static async update({ id, input }) {
    const movieIndex = movies.findIndex((movie) => movie.id === id);
    if (movieIndex === -1) return false;

    movies[movieIndex] = {
      ...movies[movieIndex],
      ...input,
    };

    return movies[movieIndex];
  }

  static updatedevice(payload) {
    devices[payload.id] = payload;
  }
  static updateDeviceAccess(payload) {
    devicesAccess[payload.id] = payload;
  }
  static updatedeviceIP(payload) {
    devices[payload.id]['ip'] = payload.ip;
  }
  static updatedevicetime(id) {
    let currentTime = new Date();
    if (devices[id]) {
      devices[id]['lastUpdate'] = currentTime;
    } else {
      console.log('update delete device');
    }
  }
  static get_device_ns(uav_id) {
    return devices[uav_id].name;
  }
  static get_device_category(uav_id) {
    return devices[uav_id].category;
  }

  static removedevice({ id }) {
    delete devices[id];
    console.log(devices);
  }

  static async addAllUAV() {
    for (let device of devices_init.init) {
      let uno = await this.create(device);
    }
    console.log('finish to add all devices  ------------');
  }
}
DevicesModel.addAllUAV();
