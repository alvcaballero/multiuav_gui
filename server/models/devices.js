import {
  StreamServer,
  DEVICE_CHECK_INTERVAL_MS,
  DEVICE_UPDATE_INTERVAL_MS,
  DEVICE_TIMEOUT_MS,
} from '../config/config.js';
import { rosController } from '../controllers/ros.js';
import sequelize, { Op } from '../common/sequelize.js';
import { cameraModel } from './camera.js';
import { positionsController } from '../controllers/positions.js';
import { logger } from '../common/logger.js';
import { readDataFile } from '../common/utils.js';
import { DEVICE_STATUS } from '../config/status.js';
import { telemetryWatchdog } from './inspect/telemetryWatchdog.js';
import { categoryModel } from './category.js';
import { deviceModelsModel } from './deviceModels/deviceModels.js';

const publicFields = ['id', 'name', 'category', 'deviceModel', 'camera', 'status', 'protocol', 'attributes', 'lastUpdate'];
const privateFields = ['id', 'name', 'files'];

// File-download presets (path, folder type, delete, srvDownload) shared by every
// installation. A device entry references one by `type` and may override any of
// FILE_OVERRIDE_KEYS field-by-field (e.g. a custom `path`). Resolving the effective
// config is the DEVICE's responsibility — `files.js` only consumes the result and
// must not read this YAML nor know about presets.
const filesSetup = readDataFile('../config/devices/devices.yaml');
const FILE_OVERRIDE_KEYS = ['path', 'downloadType', 'delete', 'srvDownload'];

// Remote folder paths must end in '/' so `files.js` can concatenate filenames.
// Trims accidental whitespace and appends a trailing slash when missing.
const normalizeFolderPath = (path) => {
  if (typeof path !== 'string') return path;
  const trimmed = path.trim();
  if (trimmed === '' || trimmed.endsWith('/')) return trimmed;
  return `${trimmed}/`;
};

const protocols = Object.freeze({
  ROS: 'ros',
  ROBOFLEET: 'robofleet',
  MAVLINK: 'mavlink',
});

class DeviceHealthMonitor {
  constructor() {
    this._updateTimer = setInterval(() => this._updateDeviceTime(), DEVICE_UPDATE_INTERVAL_MS);
    this._checkTimer = setInterval(() => this._checkDeviceOnline(), DEVICE_CHECK_INTERVAL_MS);
    logger.info('DeviceHealthMonitor started');

    process.on('SIGTERM', () => this.stop());
    process.on('SIGINT', () => this.stop());
  }

  stop() {
    clearInterval(this._updateTimer);
    clearInterval(this._checkTimer);
    this._updateTimer = null;
    this._checkTimer = null;
    logger.info('DeviceHealthMonitor stopped');
  }

  async _updateDeviceTime() {
    const limitDate = new Date(Date.now() - DEVICE_TIMEOUT_MS);
    try {
      const updates = await positionsController.getLastPositions();
      const validUpdates = updates.filter((update) => new Date(update.deviceTime) > limitDate);

      if (validUpdates.length > 0) {
        const transaction = await sequelize.transaction();
        try {
          await Promise.all(
            validUpdates.map((update) =>
              sequelize.models.Device.update(
                { lastUpdate: update.deviceTime, status: DEVICE_STATUS.ONLINE },
                { where: { id: update.deviceId }, transaction }
              )
            )
          );
          await transaction.commit();
        } catch (error) {
          await transaction.rollback();
          logger.error('Error al actualizar dispositivos:', error);
        }
      }
    } catch (error) {
      logger.error('Error en updateDeviceTime:', error);
    }
  }

  async _checkDeviceOnline() {
    const cutoffTime = new Date(Date.now() - DEVICE_TIMEOUT_MS);
    try {
      await sequelize.models.Device.update(
        { status: DEVICE_STATUS.OFFLINE },
        { where: { lastUpdate: { [Op.lte]: cutoffTime }, deletedAt: null } }
      );
    } catch (error) {
      logger.error('Error en checkDeviceOnline:', error);
    }
  }
}

export const deviceHealthMonitor = new DeviceHealthMonitor();

export class DevicesModel {
  constructor() {
    // conect with ros and other things
  }

  static async getAll(query) {
    let mydevices = await sequelize.models.Device.findAll({
      attributes: publicFields,
      where: { deletedAt: null },
    });
    // `telemetry`: the slots that stopped (or never started) arriving — see telemetryWatchdog.js
    // `platform`: what the client draws it as (vehicle type, icon, 3D model), from its device model.
    const plain = mydevices.map((d) => ({
      ...d.dataValues,
      telemetry: telemetryWatchdog.statusOf(d.id),
      platform: deviceModelsModel.platformOf(d.deviceModel),
    }));
    if (query) {
      if (Array.isArray(query)) {
        return plain.filter((device) => query.some((element) => device.id == element));
      }
      if (!isNaN(query)) {
        return plain.filter((device) => device.id == query);
      }
    }
    return plain;
  }

  static async getDevicesWithPositions() {
    const [devices, positions] = await Promise.all([DevicesModel.getAll(), positionsController.getLastPositions()]);

    const positionsByDeviceId = Object.fromEntries(positions.map((p) => [p.deviceId, p]));

    return Promise.all(
      devices.map(async (device) => {
        const pos = positionsByDeviceId[device.id];
        const positionInfo = { yaw: pos?.course };
        if (pos?.latitude !== undefined) {
          positionInfo.latitude = pos.latitude;
          positionInfo.longitude = pos.longitude;
          positionInfo.altitude = pos.altitude;
        }
        if (pos?.attributes?.localposition !== undefined) {
          positionInfo.localposition = {
            x: pos.attributes.localposition[0],
            y: pos.attributes.localposition[1],
            z: pos.attributes.localposition[2],
          };
        }
        if (pos?.attributes?.batteryLevel !== undefined) {
          positionInfo.batteryLevel = pos.attributes.batteryLevel;
        }

        const actionsByKey = rosController.getActionStatusByName(device.name);
        const isBusy =
          typeof actionsByKey.status === 'string'
            ? actionsByKey.status === 'idle'
              ? false
              : actionsByKey.status === 'executing' || actionsByKey.status === 'canceling'
            : Object.values(actionsByKey).some((a) => a.status === 'executing' || a.status === 'canceling');

        return {
          id: device.id,
          name: device.name,
          connection_status: device.status,
          busy: isBusy,
          lastUpdate: device.lastUpdate,
          ...positionInfo,
        };
      })
    );
  }

  static async getById({ id }) {
    return await sequelize.models.Device.findOne({
      where: { id: id, deletedAt: null },
    });
  }
  static async getByName(name) {
    return await sequelize.models.Device.findOne({
      attributes: publicFields,
      where: { name: name, deletedAt: null },
    });
  }

  // Every mavlink device with this system id: more than one is a misconfiguration the caller reports.
  static async getBySystemId(systemId) {
    // Filtered here, not in SQL: JSON paths in a WHERE differ per dialect and a fleet is small.
    const mavlink = await sequelize.models.Device.findAll({
      attributes: publicFields,
      where: { protocol: protocols.MAVLINK, deletedAt: null },
    });
    return mavlink.filter((d) => Number(d.attributes?.systemId) === Number(systemId));
  }

  static async getAccess(id) {
    return await sequelize.models.Device.findOne({
      attributes: privateFields,
      where: { id: id, deletedAt: null },
    });
  }

  /*
   / Resolve the effective file-download config for every source of a device.
   / Each `device.files` entry references a preset by `type`; the entry may then
   / override any of FILE_OVERRIDE_KEYS. `downloadType` (device) maps onto the
   / preset's `type` (folder mode: all/lastFolder/specific) so it doesn't collide
   / with `type` (the preset key). Returns an array of self-contained configs —
   / `files.js` iterates them and needs nothing else to connect/list/download.
   */
  static async getFilesConfig(uavId) {
    const device = await this.getAccess(uavId);
    const deviceFiles = device?.files ?? [];
    if (deviceFiles.length === 0) {
      logger.warn(`Device ${uavId} has no files setup`);
      return [];
    }

    return deviceFiles.map((entry) => {
      const preset = filesSetup.files?.[entry.type] ?? filesSetup.files?.default ?? {};
      const config = { ...preset };
      for (const key of FILE_OVERRIDE_KEYS) {
        if (entry[key] === undefined) continue;
        // `downloadType` overrides the folder mode, stored as `type` in the config.
        if (key === 'downloadType') config.type = entry.downloadType;
        else config[key] = entry[key];
      }
      // `files.js` concatenates `${path}${file}` assuming a trailing slash, so
      // normalize it here — a hand-typed custom path (e.g. './uav_media/uav_1')
      // would otherwise glue onto the filename ('./uav_media/uav_1foto.jpg').
      config.path = normalizeFolderPath(config.path);
      config.url = entry.url;
      config.preset = entry.type;
      return config;
    });
  }

  static async create(device) {
    let myDevice = null;
    let serverState = rosController.getServerStatus();
    let protocol = device.protocol ? device.protocol : protocols.ROS;

    try {
      myDevice = await sequelize.models.Device.create({
        name: device.name,
        category: device.category,
        deviceModel: device.deviceModel,
        status: DEVICE_STATUS.OFFLINE,
        camera: device.camera,
        files: device.files,
        protocol: protocol,
        attributes: device.attributes ?? {},
      });
      logger.info(`Device created: ${myDevice.id}, ${myDevice.name}, ${myDevice.category}`);
    } catch (e) {
      logger.error('Error create device: ' + e.name);
      if (e.name === 'SequelizeUniqueConstraintError') {
        return { state: 'error', msg: 'Device already exists' };
      }
      return { state: 'error', msg: 'Error create device' };
    }

    if (StreamServer) {
      cameraModel.addCameraWebRTC(device);
    }

    // A MAVLink device has nothing to subscribe in ROS: MavlinkServer matches it by systemId.
    if (protocol === protocols.MAVLINK) {
      logger.info(`Device ${device.name} added. Type: ${device.category}, MAVLink sysid ${device.attributes?.systemId}`);
      return { state: 'success', msg: 'Device created' };
    }

    if (serverState.state === 'connect') {
      logger.debug('suscribe devices');
      await rosController.subscribeDevice({
        id: myDevice.id,
        name: myDevice.name,
        category: myDevice.category,
        camera: device.camera,
        watch_bound: true,
        bag: false,
      });

      logger.info(`Device ${device.name} added. Type: ${device.category}`);
      return { state: 'success', msg: 'conectado Correctamente' };
    } else {
      logger.warn('ROS not connected. Please connect first.');
      return { state: 'error', msg: 'Ros no está conectado' };
    }
  }

  static async delete({ id }) {
    let device = await this.getById({ id: id });
    logger.info(`Removing device id=${id}`);
    await cameraModel.removeCameraWebRTC(device);
    await this.removedevice({ id: id });
    let response = await rosController.unsubscribeDevice(id);
    return response;
  }

  static async editDevice({ id, name, category, deviceModel, camera, files, protocol, attributes }) {
    let myDevice = await sequelize.models.Device.findOne({ where: { id: id }, raw: false });
    // Already checked against the category it leaves with by the controller.
    if (deviceModel && deviceModel !== myDevice.deviceModel) {
      logger.debug(`Device ${id}: changing device model to ${deviceModel}`);
      myDevice.deviceModel = deviceModel;
    }
    if (protocol && protocol !== myDevice.protocol) {
      logger.debug(`Device ${id}: changing protocol to ${protocol}`);
      myDevice.protocol = protocol;
    }
    // Already validated against the protocol's schema (and merged into the stored ones) by the controller.
    if (attributes !== undefined && JSON.stringify(attributes) !== JSON.stringify(myDevice.attributes)) {
      logger.debug(`Device ${id}: changing protocol attributes`);
      myDevice.attributes = attributes;
    }
    if ((name && name !== myDevice.name) || (category && category !== myDevice.category)) {
      logger.debug(`Device ${id}: changing name/category`);
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
          camera: myDevice.camera,
        });
      }
    }
    if (camera && JSON.stringify(camera) !== JSON.stringify(myDevice.camera)) {
      logger.debug(`Device ${id}: changing camera config`);
      myDevice.camera = camera;
      cameraModel.removeCameraWebRTC(myDevice);
      cameraModel.addCameraWebRTC({ ...myDevice, camera: camera });
    }

    if (files) myDevice.files = files;

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

  static async removedevice({ id }) {
    await sequelize.models.Device.update({ deletedAt: new Date() }, { where: { id: id } });
  }

  // A device from before device models (the column was added empty) gets its category's default,
  // once, and says so; one whose model is unknown or no longer fits its category is reported (it
  // keeps working: the model only says what it is, not how it is talked to).
  static async #checkDeviceModel(device) {
    if (!device.deviceModel) {
      const deviceModel = deviceModelsModel.defaultFor(device.category);
      await sequelize.models.Device.update({ deviceModel }, { where: { id: device.id } });
      logger.info(`Device ${device.name} (id=${device.id}): no device model, set to its category's default '${deviceModel}'`);
      return;
    }
    const problem = deviceModelsModel.deviceProblem(device);
    if (problem) logger.warn(`Device ${device.name} (id=${device.id}): ${problem.message}`);
  }

  static async addAllUAV() {
    const myDevices = await sequelize.models.Device.findAll({
      attributes: ['id', 'name', 'category', 'deviceModel', 'camera', 'status', 'protocol'],
      where: { deletedAt: null },
    });
    for (let device of myDevices) {
      // A deleted/renamed category leaves the device with no subscriptions at all (ros.js
      // resolves its config to undefined): say it once at boot instead of staying silent.
      if (!categoryModel.getCategory(device.category)) {
        logger.error(`Device ${device.name} (id=${device.id}) has category "${device.category}", which does not exist: it gets no telemetry`);
      } else {
        await DevicesModel.#checkDeviceModel(device);
      }
      if (StreamServer) {
        await cameraModel.addCameraWebRTC(device);
      }
    }
  }
}

DevicesModel.addAllUAV();
