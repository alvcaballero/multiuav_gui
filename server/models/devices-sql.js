import { readJSON, readYAML, getDatetime } from '../common/utils.js';
import { StreamServer } from '../config/config.js';
import { rosController } from '../controllers/ros.js';
import sequelize, { Op } from '../common/sequelize.js';
import { object, set } from 'zod';

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
const CHECK_INTERVAL = 5000;

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

console.log('load model devices SQL');

const CheckDeviceOnline = async () => {
  await sequelize.models.Device.update(
    { status: devicesStatus.OFFLINE }, {
    where: { lastUpdate: { [Op.lte]: new Date(new Date - 30000) } },
  });

  setTimeout(CheckDeviceOnline, CHECK_INTERVAL);
}
setTimeout(CheckDeviceOnline, CHECK_INTERVAL);

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
        const filtered = mydevices.filter((device) => query.some((element) => device.id == element));
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

    console.log('create UAV ' + uav_ns + '  type' + uav_type);

    let repeat_device = false;
    let devices = await this.getAll();
    if (devices.length > 0) {
      devices.forEach((element) => {
        repeat_device = element.name == uav_ns ? true : false;
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
        category: uav_type,
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
        await fetch(`http://localhost:9997/v3/config/paths/add/${device.name}_${device.camera[i].source}`, {
          method: 'POST',
          body: JSON.stringify({
            source: `rtsp://${device.ip}:8554/${device.camera[i].source}`,
          }),
          headers: {
            'Content-Type': 'application/json',
          },
        });
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

  static updatedevice(payload) {
    const myDevice = sequelize.models.Device.findOne({ where: { id: payload.id } });
    if (myDevice) {
      myDevice = { ...myDevice, ...payload };
    }
  }

  static updateDeviceAccess(payload) {
    const myDevice = sequelize.models.Device.findOne({ where: { id: payload.id } });
    if (myDevice) {
      myDevice = { ...myDevice, ...payload };
    }
  }
  static updatedeviceIP(payload) {
    const myDevice = sequelize.models.Device.update({ ip: payload.ip }, { where: { id: payload.id } });
  }
  static updateDeviceTime(id) {
    let currentTime = new Date();
    const myDevice = sequelize.models.Device.update({ lastUpdate: currentTime }, { where: { id: id } });
    if (myDevice) {
      return myDevice;
    } else {
      console.log('update delete device');
      return false;
    }
  }

  static get_device_ns(uav_id) {
    const myDevice = sequelize.models.Device.findOne({ where: { id: uav_id } });
    return myDevice.name;
  }
  static get_device_category(uav_id) {
    const myDevice = sequelize.models.Device.findOne({ where: { id: uav_id } });
    return myDevice.category;
  }

  static removedevice({ id }) {
    sequelize.models.Device.destroy({ where: { id: id } });
    console.log(devices);
  }

  static async addAllUAV() {
    console.log('---- init cameras of devices ------------');
    const myDevices = await this.getAll();
    for (let device of myDevices) {
      if (StreamServer) {
        await this.addCameraWebRTC(device);
      }
    }
  }
}

DevicesModel.addAllUAV();
