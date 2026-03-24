import { devicesController } from '../../controllers/devices.js';
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
} from './rosSubscriptions.js';
import * as rosServices from './rosServices.js';

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
    return _subscribeDevice(uavAdded, getRos(), serverStatus());
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
    return rosServices.getTopics(getRos());
  }

  static getServices() {
    return rosServices.getServices(getRos());
  }

  static async getServicesType(service) {
    return rosServices.getServicesType(service, getRos());
  }

  static async getServiceRequestDetails(type) {
    return rosServices.getServiceRequestDetails(type, getRos());
  }

  static async getServiceResponseDetails(type) {
    return rosServices.getServiceResponseDetails(type, getRos());
  }

  static getTopicType(topic) {
    return rosServices.getTopicType(topic, getRos());
  }

  static getMessageDetails(message) {
    return rosServices.getMessageDetails(message, getRos());
  }

  static async getRosVersion() {
    return rosServices.getRosVersion(getRos());
  }

  static async getPublishers(topic) {
    return rosServices.getPublishers(topic, getRos());
  }

  static async PubRosMsg(params) {
    return rosServices.PubRosMsg(params, getRos());
  }

  static async subscribeOnce({ topic, messageType, timeout = 2000 }) {
    return rosServices.subscribeOnce({ topic, messageType, timeout }, getRos());
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
    return rosServices.getActionServer(getRos());
  }

  static async getActionGoalmsg(actionServer) {
    return rosServices.getActionGoalmsg(actionServer, getRos());
  }

  static async sendActionGoal(args) {
    return rosServices.sendActionGoal(args, getRos());
  }

  static async cancelActionGoal(args) {
    return rosServices.cancelActionGoal(args, getRos());
  }

  static async getActionServers() {
    return rosServices.getActionServers(getRos());
  }

  static Getservicehost(nameService) {
    return rosServices.Getservicehost(nameService, getRos());
  }

  static async getListMaster() {
    return rosServices.getListMaster(getRos());
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
