import { readJSON, readYAML, getDatetime } from '../common/utils.js';
import { StreamServer } from '../config/config.js';
import { rosController } from '../controllers/ros.js';
import sequelize from '../common/sequelize.js';
import { object } from 'zod';

const devices_init = readYAML('../config/devices/devices_init.yaml');
const devices = {};
const devicesAccess = {};
const listUpdateTime = {};

console.log('load model devices SQL');

const CheckDeviceOnline = setInterval(async () => {
  let currentTime = new Date();
  let checkdevices = Object.keys(devices);
  let mydevices = await sequelize.models.Device.findAll({
    attributes: ['id', 'status', 'lastUpdate'],
  });

  Object.values(listUpdateTime).forEach((element) => {
    if (element.flag) {
      sequelize.models.Device.update(
        { lastUpdate: element.time },
        { where: { id: element.deviceId } }
      );
      listUpdateTime[element.deviceId].flag = false;
    }
  });

  mydevices.forEach((element) => {
    if (element['status'] == 'online' && currentTime - element['lastUpdate'] > 30000) {
      sequelize.models.Device.update({ status: 'offline' }, { where: { id: element.id } });
    }
    if (element['status'] == 'offline' && currentTime - element['lastUpdate'] < 30000) {
      sequelize.models.Device.update({ status: 'online' }, { where: { id: element.id } });
    }
  });
}, 5000);

export class DevicesModel {
  constructor() {
    // conect with ros and other things
    console.log('constructor device model');
  }

  static async getAll(query) {
    let mydevices = await sequelize.models.Device.findAll({
      attributes: ['id', 'name', 'category', 'ip', 'camera', 'status', 'lastUpdate'],
    });
    if (query) {
      console.log(query);
      if (Array.isArray(query)) {
        const filtered = mydevices.filter((device) =>
          query.some((element) => device.id == element)
        );
        return Object.fromEntries(filtered);
      }
      if (!isNaN(query)) {
        const filtered = mydevices.filter((device) => device.id == query);
        return filtered ? { id: filtered } : {};
      }
    }
    return mydevices;
  }
  static async getAccess(id) {
    return await sequelize.models.Device.findAll({
      attributes: ['id', 'name', 'user', 'pwd', 'ip'],
    });
  }

  static async create(device) {
    console.log(device);
    let serverState = rosController.getServerStatus();

    let uav_ns = device.name;
    let uav_type = device.category;
    let cur_uav_idx = String(Object.values(devices).length);

    console.log('create UAV ' + uav_ns + '  type' + uav_type);

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
    const myDevice = await sequelize.models.Device.create({
      name: device.name,
      category: device.category,
      ip: device.ip,
      status: 'offline',
      camera: JSON.stringify(device.camera),
      user: device.user,
      pwd: device.pwd,
      ip: device.ip,
    });
    if (StreamServer) {
      this.addCameraWebRTC(device);
    }

    if (serverState.state === 'connect') {
      console.log('suscribe devices ');
      await rosController.subscribeDevice({
        id: myDevice.id,
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
    return await sequelize.models.Device.findOne({ where: { id: id } });
  }
  static async getByName(name) {
    return await sequelize.models.Device.findOne({ where: { name: name } });
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
  static updateDeviceTime(id) {
    let currentTime = new Date();
    listUpdateTime[id] = { deviceId: id, time: currentTime, flag: true };
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
    console.log('---- start init devices ------------');
    let NumDevices = 0;
    try {
      NumDevices = await sequelize.models.Device.count();
    } catch (error) {
      console.error('Error fetching devices:', error);
      throw error;
    }
    if (NumDevices <= 0) {
      console.log('---- Dont have devices in data base add new from file ------------');
      for (let device of devices_init.init) {
        await this.create(device);
      }
    } else {
      let mydevices = await this.getAll();
      if (StreamServer) {
        for (let device of mydevices) {
          await this.addCameraWebRTC(device);
        }
      }
    }
    console.log('------  finish init devices ------------');
  }
}

DevicesModel.addAllUAV();
