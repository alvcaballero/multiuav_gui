import { devicesController } from '../../controllers/devices.js';
import { positionsController } from '../../controllers/positions.js';
import { decodeRosMsg } from './rosDecode.js';
import {
  getRos,
  setRosState,
  serverStatus,
  rosConnect,
  disconectRos as _disconectRos,
  initAutoConnect,
} from './rosConnection.js';
import {
  subscribeDevice as _subscribeDevice,
  unsubscribeDevice as _unsubscribeDevice,
  RosSubscribe as _RosSubscribe,
  RosSubscribeCamera as _RosSubscribeCamera,
  PubRosMsg as _PubRosMsg,
  subscribeOnce as _subscribeOnce,
} from './rosTopics.js';
import * as rosServices from './rosServices.js';
import * as rosInspect from './rosInspect.js';
import * as actionRegistry from './rosActionRegistry.js';

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

  // Subscribing
  static RosSubscribe(uav_id, uav_type, type, msgType, callback) {
    _RosSubscribe(uav_id, uav_type, type, msgType, callback, getRos());
  }

  static RosSubscribeCamera(uav_id, uav_type, type, msgType, callback) {
    _RosSubscribeCamera(uav_id, uav_type, type, msgType, callback, getRos());
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
    return rosServices.callService({ uav_id, type, request }, getRos());
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
    rosServices.GCSServicesMission(getRos());
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

  static async sendActionGoal(args) {
    return actionRegistry.sendActionGoal(args, getRos());
  }

  static getActionStatus(params) {
    return actionRegistry.getActionStatus(params);
  }

  static cancelAction(params) {
    return actionRegistry.cancelAction(params);
  }

  static async cancelActionGoal(args) {
    return rosServices.cancelActionGoal(args, getRos());
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
    await rosModel.connectAllUAV();
    rosModel.GCSServicesMission();
  },
  () => {
    rosModel.unsubscribeDevice(-1);
    rosModel.GCSunServicesMission();
  }
);
