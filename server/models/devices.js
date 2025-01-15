import { readJSON, readYAML, getDatetime, writeJSON } from '../common/utils.js';
import { StreamServer } from '../config/config.js';
import { rosController } from '../controllers/ros.js';
import { object } from 'zod';
import { devicesData } from '../config/config.js';

/* devices:
/   id
/   name  : name of uav
/   category : model of uav registered
/   ip 
/   protocol: ros, robofleet
/   camera : array of camera devices 
/       type: WebRTC, RTSP
/       source: source of camera
/   files: array of files access
/        url: ftp://user:pwd@ip:port
/        type: ftp
/   lastUpdate:
/   status:
*/
const devices = readJSON(devicesData);
const CHECK_INTERVAL = 5000;

console.log('load model devices No SQL');
const publicFields = ['id', 'name', 'category', 'camera', 'status', 'protocol', 'lastUpdate'];
const privateFields = ['id', 'name', 'user', 'pwd', 'ip', 'files'];

const devicesStatus = Object.freeze({
  ONLINE: 'online',
  OFFLINE: 'offline',
});

function filterDevice(obj, fields) {
  let newObj = {};
  for (let field of fields) {
    if (obj.hasOwnProperty(field)) {
      newObj[field] = obj[field] ?? null;
    }
  }
  return newObj;
}

const checkDeviceOnline = () => {
  const currentTime = new Date();
  const checkDevices = Object.keys(devices);

  checkDevices.forEach((element) => {
    const { lastUpdate } = devices[element];
    devices[element].status = currentTime - lastUpdate < CHECK_INTERVAL ? devicesStatus.ONLINE : devicesStatus.OFFLINE;
  });

  setTimeout(checkDeviceOnline, CHECK_INTERVAL);
};

setTimeout(checkDeviceOnline, CHECK_INTERVAL);

export class DevicesModel {
  constructor() {
    console.log('constructor device model');
  }

  static async getAll(query) {
    if (query) {
      console.log(query);
      if (Array.isArray(query)) {
        const asArray = Object.entries(devices);
        const filtered = asArray.filter(([key, value]) => query.some((element) => key == element));
        let result = {};
        for (let [key, value] of filtered) {
          result[key] = filterDevice(value, publicFields);
        }
        return result;
      }
      if (!isNaN(query)) {
        return devices[query] ? { id: filterDevice(devices[query], publicFields) } : {};
      }
    }
    let listDevices = Object.entries(devices);
    let result = {};
    for (let [key, value] of listDevices) {
      result[key] = filterDevice(value, publicFields);
    }
    return result;
  }
  static async getAccess(id) {
    return filterDevice(
      Object.values(devices).find((device) => device.id === id),
      privateFields
    );
  }

  static async create(device) {
    let serverState = rosController.getServerStatus();

    let cur_uav_idx = String(Object.values(devices).length);

    let protocol = device.protocol ? device.protocol : 'ros';

    console.log(device);

    console.log(`create UAV  ${device.name} type: ${device.category} protocol ${protocol} server ${serverState.state}`);

    let repeat_device = false;
    if (Object.values(devices).length > 0) {
      Object.values(devices).forEach((element) => {
        if (element) {
          repeat_device = element.name == device.name ? true : false;
        }
      });
    }

    if (repeat_device == true) {
      console.log('Dispositivo ya se encuentra registrado ' + device.name);
      return {
        state: 'error',
        msg: `Dispositivo ya se encuentra registrado ${device.name}`,
      };
    }
    let repeatId = false;
    do {
      repeatId = devices.hasOwnProperty(cur_uav_idx) ? true : false;
      if (repeatId) {
        cur_uav_idx = String(Number(cur_uav_idx) + 1);
      }
    } while (repeatId);

    let otherFields = { id: cur_uav_idx, status: 'offline', lastUpdate: null, protocol: protocol };

    this.updatedevice({
      ...device,
      ...otherFields,
    });

    if (StreamServer) {
      this.addCameraWebRTC(device);
    }

    if (serverState.state === 'connect') {
      if (protocol == 'ros') {
        console.log('suscribe devices ');
        await rosController.subscribeDevice({
          id: cur_uav_idx,
          name: device.name,
          category: device.category,
          camera: device.camera,
          watch_bound: true,
          bag: false,
        });
      }
      console.log('success create', device.name + ' Type: ' + device.category);
      return { state: 'success', msg: 'conectado Correctamente' };
    } else {
      console.log('success create', device.name + ' Type: ' + device.category + ' -Ros dont connect ');
      return { state: 'error', msg: 'Ros no está conectado' };
    }
  }

  static async addCameraWebRTC(device) {
    const devicePort = device.ip === '127.0.0.1' ? 8553 : 8554;
    for (let i = 0; i < device.camera.length; i = i + 1) {
      if (device.camera[i]['type'] == 'WebRTC') {
        try {
          let response = await fetch(
            `http://localhost:9997/v3/config/paths/add/${device.name}_${device.camera[i].source}`,
            {
              method: 'POST',
              body: JSON.stringify({
                source: `rtsp://${device.ip}:${devicePort}/${device.camera[i].source}`,
              }),
              headers: {
                'Content-Type': 'application/json',
              },
            }
          );
          if (response.status == 200) {
            console.log('camera added ' + device.camera[i].source);
          } else {
            console.log(`Error adding camera  ${response.status} ${device.camera[i].source}`);
            return false;
          }
        } catch (e) {
          console.log('\x1b[31m%s\x1b[0m', 'Error adding camera ' + device.camera[i].source);
          return false;
        }
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

  static updatedevice(payload) {
    const deviceIndex = devices.findIndex((device) => device.id === payload.id);
    if (deviceIndex === -1) return false;

    devices[payload.id] = { ...devices[payload.id], ...payload };
    writeJSON(devicesData, devices);
  }

  static updatedeviceIP(payload) {
    devices[payload.id]['ip'] = payload.ip;
  }
  static updateDeviceTime(id) {
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
    //console.log(devices);
  }
  static async addAllUAV() {
    console.log('---- init cameras of devices ------------');
    for (let device of Object.values(devices)) {
      if (StreamServer) {
        await this.addCameraWebRTC(device);
      }
    }
  }
}

DevicesModel.addAllUAV();
