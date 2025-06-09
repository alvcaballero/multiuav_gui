import { readJSON, readYAML, getDatetime } from '../common/utils.js';
import { StreamServer } from '../config/config.js';
import { rosController } from '../controllers/ros.js';
import sequelize, { Op } from '../common/sequelize.js';
import { cameraModel } from './camera.js';
import { object, set } from 'zod';
import { positionsController } from '../controllers/positions.js';
/* devices:
/   id
/   name  : name of uav
/   category : model of uav registered
/   ip : ip of uav
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

const protocols = Object.freeze({
  ROS: 'ros',
  ROBOFLEET: 'robofleet',
});

const updateDeviceTime = async () => {
  const transactions = await sequelize.transaction();
  const updates = await positionsController.getLastPositions();
  if (updates.length > 0) {
    try {
      const transactions = await sequelize.transaction();
      const promises = updates.map((update) => {
        return sequelize.models.Device.update(
          { lastUpdate: update.deviceTime, status: devicesStatus.ONLINE },
          {
            where: { id: update.deviceId },
            transaction: transactions,
          }
        );
      });

      await Promise.all(promises);
      await transactions.commit();
    } catch (error) {
      if (transactions) await transactions.rollback();
      console.error('Error al actualizar dispositivos:', error);
    }
  }
  setTimeout(updateDeviceTime, 1500);
};
setTimeout(updateDeviceTime, 1500);

const CheckDeviceOnline = async () => {
  await sequelize.models.Device.update(
    { status: devicesStatus.OFFLINE },
    {
      where: { lastUpdate: { [Op.lte]: new Date(new Date() - 30000) } },
    }
  );

  setTimeout(CheckDeviceOnline, CHECK_INTERVAL);
};
setTimeout(CheckDeviceOnline, CHECK_INTERVAL);

export class DevicesModel {
  constructor() {
    // conect with ros and other things
    console.log('constructor device model');
  }

  static async getAll(query) {
    let mydevices = await sequelize.models.Device.findAll({
      attributes: publicFields,
    });
    if (query) {
      console.log(query);
      if (Array.isArray(query)) {
        return mydevices.filter((device) => query.some((element) => device.id == element));
      }
      if (!isNaN(query)) {
        return mydevices.filter((device) => device.id == query);
      }
    }
    return mydevices;
  }

  static async getById({ id }) {
    return await sequelize.models.Device.findOne({
      where: { id: id },
    });
  }
  static async getByName(name) {
    return await sequelize.models.Device.findOne({
      attributes: publicFields,
      where: { name: name },
    });
  }

  static async getAccess(id) {
    return await sequelize.models.Device.findOne({
      attributes: privateFields,
      where: { id: id },
    });
  }

  static async create(device) {
    let myDevice = null;
    let serverState = rosController.getServerStatus();

    let protocol = device.protocol ? device.protocol : protocols.ROS;

    console.log('create UAV ' + device.name + '  type' + device.category);
    try {
      myDevice = await sequelize.models.Device.create({
        name: device.name,
        category: device.category,
        ip: device.ip,
        status: devicesStatus.OFFLINE,
        user: device.user,
        pwd: device.pwd,
        ip: device.ip,
        camera: device.camera,
        files: device.files,
        protocol: protocol,
      });
    } catch (e) {
      console.log('Error create device' + e.name);
      console.log(e);
      if (e.name === 'SequelizeUniqueConstraintError') {
        return { state: 'error', msg: 'Device already exists' };
      }
      return { state: 'error', msg: 'Error create device' };
    }

    if (StreamServer) {
      cameraModel.addCameraWebRTC(device);
    }

    if (serverState.state === 'connect') {
      console.log('suscribe devices ');
      await rosController.subscribeDevice({
        id: myDevice.id,
        name: myDevice.name,
        category: myDevice.category,
        camera: device.camera,
        watch_bound: true,
        bag: false,
      });

      console.log('success', device.name + ' added. Type: ' + device.category);
      return { state: 'success', msg: 'conectado Correctamente' };
    } else {
      console.log('\nRos no está conectado.\n\n Por favor conéctelo primero.');
      return { state: 'error', msg: 'Ros no está conectado' };
    }
  }

  static async delete({ id }) {
    let device = await this.getById({ id: id });
    console.log('remove id' + id);
    await cameraModel.removeCameraWebRTC(device);
    await this.removedevice({ id: id });
    let response = await rosController.unsubscribeDevice(id);
    return response;
  }

  static async editDevice({ id, name, category, ip, user, pwd, camera, files, protocol }) {
    let myDevice = await sequelize.models.Device.findOne({ where: { id: id }, raw: false });
    if (protocol && protocol !== myDevice.protocol) {
      console.log('change protocol');
      myDevice.protocol = protocol;
    }
    if ((name && name !== myDevice.name) || (category && category !== myDevice.category)) {
      console.log('change name');
      myDevice.name = name ? name : myDevice.name;
      myDevice.category = category ? category : myDevice.category;
      if (protocol === protocols.ROBOFLEET) {
        // unsuscribe topics
        // subscribe new topics
      }
      if (protocol === protocols.ROS) {
        // unsuscribe topics
        await rosController.unsubscribeDevice(myDevice.id);
        // subscribe new topics
        await rosController.subscribeDevice({
          id: myDevice.id,
          name: myDevice.name,
          category: myDevice.category,
          camera: myDevice.camerak,
        });
      }
    }
    if (ip && ip !== myDevice.ip) {
      cameraModel.removeCameraWebRTC(myDevice);
      myDevice.ip = ip;
      cameraModel.removeCameraWebRTC({ ...myDevice, ip: ip });
    }

    if (camera && JSON.stringify(camera) !== JSON.stringify(myDevice.camera)) {
      console.log('change camera');
      myDevice.camera = camera;
      cameraModel.removeCameraWebRTC(myDevice);
      cameraModel.addCameraWebRTC({ ...myDevice, camera: camera });
    }

    if (files) myDevice.files = files;
    if (user) myDevice.user = user;
    if (pwd) myDevice.pwd = pwd;

    myDevice.save();
  }

  static async get_device_ns(uav_id) {
    const myDevice = await sequelize.models.Device.findOne({ where: { id: uav_id } });
    return myDevice.name;
  }
  static async get_device_category(uav_id) {
    const myDevice = await sequelize.models.Device.findOne({ where: { id: uav_id } });
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
        await cameraModel.addCameraWebRTC(device);
      }
    }
  }
}

DevicesModel.addAllUAV();
