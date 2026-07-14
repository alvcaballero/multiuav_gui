import { devicesController } from '../../controllers/devices.js';
import { missionController } from '../../controllers/mission.js';
import { positionsController } from '../../controllers/positions.js';
import { decodeRosMsg } from './rosDecode.js';
import { logger, rosLogger } from '../../common/logger.js';
import {
  getRos,
  setRosState,
  serverStatus,
  rosConnect,
  disconectRos as _disconectRos,
  initAutoConnect,
  getRosVersionInfo as _getRosVersionInfo,
  setRosVersionInfo,
} from './rosConnection.js';
import {
  subscribeDevice as _subscribeDevice,
  unsubscribeDevice as _unsubscribeDevice,
  PubRosMsg as _PubRosMsg,
  publishTopic as _publishTopic,
  subscribeOnce as _subscribeOnce,
} from './rosTopics.js';
import * as rosServices from './rosServices.js';
import * as rosInspect from './rosInspect.js';
import * as actionRegistry from './rosAction.js';

export class rosModel {
  static setrosState({ state, msg }) {
    setRosState({ state, msg });
  }

  static serverStatus() {
    return serverStatus();
  }

  static async connectAllUAV() {
    const devices = await devicesController.getAllDevices();
    for (let device of Object.values(devices)) {
      if (device.protocol == 'ros') {
        await rosModel.subscribeDevice({
          id: device.id,
          name: device.name,
          category: device.category,
          camera: device.camera,
          watch_bound: true,
          bag: false,
        });
      }
    }
  }

  static disconectRos() {
    _disconectRos(null);
  }

  static async rosConnect() {
    rosConnect(null);
  }

  static async subscribeDevice(uavAdded) {
    return _subscribeDevice(uavAdded, getRos(), serverStatus(), {
      onPosition: (args) => {
        const decoded = decodeRosMsg(args);
        if (decoded) positionsController.updatePosition(decoded);
      },
      onCamera: (args) => {
        const decoded = decodeRosMsg(args);
        if (decoded) positionsController.updateCamera(decoded);
      },
    });
  }

  static async unsubscribeDevice(id) {
    return _unsubscribeDevice(id);
  }

  static async callRosService({ service, messageType, message }) {
    return rosServices.callRosService({ service, messageType, message }, getRos());
  }

  static async callService({ uav_id, type, request }) {
    const device = await devicesController.getDevice(uav_id);
    const { name, category } = device;
    return rosServices.callService({ name, category, type, request }, getRos());
  }

  static async publishTopic({ uav_id, type, message }) {
    const device = await devicesController.getDevice(uav_id);
    const { name, category } = device;
    return _publishTopic({ name, category, type, message }, getRos());
  }

  static getTopics() {
    return rosInspect.getTopics(getRos());
  }

  static getServices() {
    return rosInspect.getServices(getRos());
  }

  static async getServicesType(service) {
    return rosInspect.getServicesType(service, getRos());
  }

  static async getServiceRequestDetails(type) {
    return rosInspect.getServiceRequestDetails(type, getRos());
  }

  static async getServiceResponseDetails(type) {
    return rosInspect.getServiceResponseDetails(type, getRos());
  }

  static getTopicType(topic) {
    return rosInspect.getTopicType(topic, getRos());
  }

  static getMessageDetails(message) {
    return rosInspect.getMessageDetails(message, getRos());
  }

  static async getRosVersion() {
    return rosInspect.getRosVersion(getRos());
  }

  static getRosVersionInfo() {
    return _getRosVersionInfo();
  }

  static async _resolveRosVersion() {
    try {
      const { version, distro } = await rosInspect.getRosVersion(getRos());
      setRosVersionInfo({ version: version ?? 1, distro: distro ?? null });
      logger.info(`ROS version resolved: ROS${version ?? 1}${distro ? ` (${distro})` : ''}`);
    } catch (error) {
      logger.warn(`Could not resolve ROS version, defaulting to ROS1: ${error.message}`);
      setRosVersionInfo({ version: 1, distro: null });
    }
  }

  static async getPublishers(topic) {
    return rosInspect.getPublishers(topic, getRos());
  }

  static async PubRosMsg(params) {
    return _PubRosMsg(params, getRos());
  }

  static async subscribeOnce({ topic, messageType, timeout = 2000 }) {
    return _subscribeOnce({ topic, messageType, timeout }, getRos());
  }

  static GCSServicesMission() {
    // Business callbacks live here in the facade — rosServices only owns the
    // ROSLIB advertise/registry lifecycle and never touches missionController.
    const gcs_services = [
      {
        name: 'ServiceFinishMission',
        serviceName: '/GCS/FinishMission',
        serviceType: 'aerialcore_common/finishMission',
        callback: function (request, response) {
          logger.debug(`Service finish mission callback: ${JSON.stringify(request)}`);
          if (request.hasOwnProperty('uav_id')) {
            missionController
              .deviceFinishMission({ name: request.uav_id })
              .catch((err) => logger.error(`deviceFinishMission failed: ${err.message}`));
          }
          Object.assign(response, { success: true, msg: 'Set successfully' });
          return true;
        },
      },
      {
        name: 'ServiceDownload',
        serviceName: '/GCS/FinishDownload',
        serviceType: 'aerialcore_common/finishGetFiles',
        callback: function (request, response) {
          logger.debug(`Service finish download files callback: ${JSON.stringify(request)}`);
          if (request.hasOwnProperty('uav_id')) {
            missionController
              .deviceFinishSyncFiles({ name: request.uav_id })
              .catch((err) => logger.error(`deviceFinishSyncFiles failed: ${err.message}`));
          }
          Object.assign(response, { success: true, msg: 'Set successfully' });
          return true;
        },
      },
      {
        name: 'GCSCommand',
        serviceName: '/GCS/GCSCommand',
        serviceType: 'muav_gcs_interfaces/srv/GCSCommand',
        callback: function (request, response) {
          logger.debug(`Service get mission callback: ${JSON.stringify(request)}`);
          if (request.hasOwnProperty('request') && request.request.hasOwnProperty('sender_ns')) {
            const command = Number(request.request.command);
            logger.debug(`GCSCommand received command "${command}" from ${request.request.sender_ns}`);
            if (command === 16) {
              missionController
                .deviceFinishMission({ name: request.request.sender_ns })
                .catch((err) => logger.error(`deviceFinishMission failed: ${err.message}`));
            } else if (command === 17) {
              missionController
                .deviceFinishSyncFiles({ name: request.request.sender_ns })
                .catch((err) => logger.error(`deviceFinishSyncFiles failed: ${err.message}`));
            } else {
              logger.debug(
                `GCSCommand unhandled command "${request.request.command}" from ${request.request.sender_ns}`
              );
            }
          }
          Object.assign(response, {
            reply: {
              timestamp: Date.now() * 1000, // o el que corresponda, microsegundos
              command: request.request.command,
              result: 0,
              result_param1: 0,
              result_param2: 0,
            },
          });
          return true;
        },
      },
    ];
    rosServices.GCSServicesMission(gcs_services, getRos());
  }

  static serviceServer({ serviceName, serviceType, callback }) {
    return rosServices.serviceServer({ serviceName, serviceType, callback }, getRos());
  }

  static GCSunServicesMission() {
    rosServices.GCSunServicesMission();
  }

  static async getActionServer() {
    return rosInspect.getActionServer(getRos());
  }

  static async getActionGoalmsg(actionServer) {
    return rosInspect.getActionGoalmsg(actionServer, getRos());
  }

  // Device-layer actions: resolve the ROS action name/type from config by uav_id.
  static async sendActionGoal({ uav_id, type, ...rest }) {
    const { name, category } = await devicesController.getDevice(uav_id);

    // CameraFileDownload finishing is a business event (mission state machine),
    // not a ROS concern — inject it here like GCSServicesMission does for services,
    // so actionRegistry/rosEncode stay unaware of missionController.
    const onComplete =
      type === 'CameraFileDownload'
        ? (result) => {
            rosLogger.debug(`sendActionGoal onComplete: ${JSON.stringify(result)}`);
            if (result.state !== 'success') return;
            missionController
              .deviceFinishSyncFiles({ name })
              .catch((err) => logger.error(`deviceFinishSyncFiles failed: ${err.message}`));
          }
        : rest.onComplete;

    return actionRegistry.sendActionGoal({ name, category, type, ...rest, onComplete }, getRos());
  }

  static async getActionStatus({ uav_id, type }) {
    const { name, category } = await devicesController.getDevice(uav_id);
    // no type → every action of the device (prefix mode, no config lookup)
    return actionRegistry.getActionStatus(type ? { name, category, type } : { name });
  }

  // Status for every action registered to a device, by name (no config lookup).
  static getActionStatusByName(name) {
    return actionRegistry.getActionStatus({ name });
  }

  static async cancelAction({ uav_id, type }) {
    const { name, category } = await devicesController.getDevice(uav_id);
    return actionRegistry.cancelAction({ name, category, type });
  }

  // Primitive-layer actions: caller supplies the fully-resolved ROS action name
  // and type (e.g. the MCP server, which already builds them).
  static async sendRosActionGoal(args) {
    return actionRegistry.sendRosActionGoal(args, getRos());
  }

  static getRosActionStatus(params) {
    return actionRegistry.getRosActionStatus(params);
  }

  static cancelRosAction(params) {
    return actionRegistry.cancelRosAction(params);
  }

  static async getActionServers() {
    return rosInspect.getActionServers(getRos());
  }

  static Getservicehost(nameService) {
    return rosInspect.Getservicehost(nameService, getRos());
  }

  static async getListMaster() {
    return rosInspect.getListMaster(getRos());
  }
}

initAutoConnect(
  async () => {
    await rosModel._resolveRosVersion();
    await rosModel.connectAllUAV();
    rosModel.GCSServicesMission();
  },
  () => {
    rosModel.unsubscribeDevice(-1);
    rosModel.GCSunServicesMission();
  }
);
