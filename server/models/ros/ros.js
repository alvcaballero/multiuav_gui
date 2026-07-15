import { devicesController } from '../../controllers/devices.js';
import { missionController } from '../../controllers/mission.js';
import { positionsController } from '../../controllers/positions.js';
import { categoryModel } from '../category.js';
import { decodeRosMsg } from './rosDecode.js';
import { logger, rosLogger, logHelpers } from '../../common/logger.js';
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
  subscribeTopics as _subscribeTopics,
  unsubscribeKey as _unsubscribeKey,
  unsubscribeAll as _unsubscribeAll,
  PubRosMsg as _PubRosMsg,
  subscribeOnce as _subscribeOnce,
} from './rosTopics.js';
import * as rosServices from './rosServices.js';
import * as rosInspect from './rosInspect.js';
import * as actionRegistry from './rosAction.js';

// Resolve a message-config block for a device category from categoryModel (the
// single source of truth — no direct devices_msg.yaml read anywhere in the ROS
// layer). Every device-layer method below resolves its config through this one
// helper, so subscribers/publishers/services/actions all share one lookup path.
//   block — 'subscribers' | 'publishers' | 'services' | 'actions'
//   type  — with it, returns that single entry (services/actions); without it,
//           the whole block (subscribers/publishers).
function resolveCategoryConfig(category, block, type) {
  const cfg = categoryModel.getCategory(category)?.[block];
  return type ? cfg?.[type] : cfg;
}

// Build the absolute ROS name for a device entry from its namespace and the
// config suffix. Device names are stored WITHOUT a leading slash (e.g. 'agv_1')
// and every config `name` starts with one (e.g. '/odom'), so the result is
// always absolute: `/agv_1/odom`. Shared by subscribers/publishers/services/
// actions so a device's topics, services and actions all resolve identically.
function buildDeviceName(name, entry) {
  return `/${name}${entry['name']}`;
}

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

  // Resolve a device's subscriber topics from the category config (categoryModel
  // is the single source of truth — no direct YAML read here), wire the business
  // effect (position vs camera) into each topic's onMessage, and hand the
  // fully-built topic list to the transport primitive. All category/camera
  // knowledge lives here — rosTopics stays business-agnostic.
  static async subscribeDevice({ id, name, category, camera = [] }) {
    logHelpers.ros.subscribe(id, name, { category });

    const subscribers = resolveCategoryConfig(category, 'subscribers');
    if (!subscribers) {
      logger.warn(`No subscribers config found for category ${category}`);
      return { state: 'error', msg: `No subscribers config for ${category}` };
    }

    // Only subscribe the camera topic when the device has a Websocket camera AND
    // the category config defines its message type — otherwise skip it.
    const hasWebsocketCamera = camera.some((cam) => cam.type == 'Websocket');
    const cameraMsgTypeDefined = Boolean(subscribers['camera']?.['messageType']);
    const shouldSubscribeCamera = hasWebsocketCamera && cameraMsgTypeDefined;
    if (hasWebsocketCamera && !cameraMsgTypeDefined) {
      logger.warn(`No message type found for camera subscription for ${name}`);
    }

    // Position vs camera differ only in which controller effect they trigger.
    // The closure captures deviceId + category so the transport never sees them.
    const makeOnMessage = (effect) => (args) => {
      const decoded = decodeRosMsg({
        msg: args.msg,
        deviceId: id,
        uav_type: category,
        type: args.slot,
        msgType: args.messageType,
      });
      if (decoded) effect(decoded);
    };
    const onPosition = makeOnMessage((d) => positionsController.updatePosition(d));
    const onCamera = makeOnMessage((d) => positionsController.updateCamera(d));

    const topics = Object.entries(subscribers)
      .filter(([slot]) => {
        if (slot == 'camera' && !shouldSubscribeCamera) {
          logger.debug(`Skipping camera subscription for ${name} as no websocket camera is present`);
          return false;
        }
        return true;
      })
      .map(([slot, cfg]) => ({
        slot,
        name: buildDeviceName(name, cfg),
        messageType: cfg['messageType'],
        onMessage: slot == 'camera' ? onCamera : onPosition,
      }));

    return _subscribeTopics({ key: id, topics }, getRos(), serverStatus());
  }

  static async unsubscribeDevice(id) {
    // id < 0 (e.g. -1 on disconnect) means "unsubscribe everything".
    return id < 0 ? _unsubscribeAll() : _unsubscribeKey(id);
  }

  // Ad-hoc subscription to an arbitrary topic, independent of any device.
  // Keyed by the topic name so it can be unsubscribed via unsubscribeTopic.
  static async subscribeTopic({ topic, messageType, onMessage }) {
    return _subscribeTopics(
      { key: topic, topics: [{ slot: 'main', name: topic, messageType, onMessage }] },
      getRos(),
      serverStatus()
    );
  }

  static async unsubscribeTopic(topic) {
    return _unsubscribeKey(topic);
  }

  static async callRosService({ service, messageType, message }) {
    return rosServices.callRosService({ service, messageType, message }, getRos());
  }

  static async callServiceDevice({ uav_id, type, request }) {
    const { name, category } = await devicesController.getDevice(uav_id);
    const serviceDef = resolveCategoryConfig(category, 'services', type);
    const service = serviceDef ? buildDeviceName(name, serviceDef) : undefined;
    return rosServices.callService(
      { name, type, service, serviceType: serviceDef?.serviceType, request },
      getRos()
    );
  }

  // Device-layer publish: resolve the publisher from the category config here
  // (mirror of subscribeDevice / callService), then delegate the fully-resolved
  // topic + messageType to the pure PubRosMsg primitive. rosTopics owns no
  // devices_msg/category knowledge.
  static async publishTopicDevice({ uav_id, type, message }) {
    const { name, category } = await devicesController.getDevice(uav_id);

    const publisherDef = resolveCategoryConfig(category, 'publishers', type);
    if (!publisherDef) {
      return { state: 'warning', msg: `${type} to ${name} dont have this publisher` };
    }

    const messageType = publisherDef['messageType'];
    const topic = buildDeviceName(name, publisherDef);
    try {
      await _PubRosMsg({ topic, messageType, message }, getRos());
      return { state: 'success', msg: `${type} to ${name} ok` };
    } catch (error) {
      const errMsg = error instanceof Error ? error.message : String(error);
      logger.error(`Error publishing topic: ${errMsg}`);
      return { state: 'error', msg: 'Failed to publish topic: ' + errMsg };
    }
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

  // Device-layer actions: resolve the action config from categoryModel by uav_id,
  // build the ROS action-server name here (single source of name-building), and
  // hand the fully-resolved name + type to the primitive.
  static async sendActionGoalDevice({ uav_id, type, ...rest }) {
    const { name, category } = await devicesController.getDevice(uav_id);
    const actionDef = resolveCategoryConfig(category, 'actions', type);
    if (!actionDef) throw new Error(`Action '${type}' not configured for device ${name}`);
    const actionServerName = buildDeviceName(name, actionDef);

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

    return actionRegistry.sendActionGoal(
      { actionServerName, actionType: actionDef.actionType, type, ...rest, onComplete },
      getRos()
    );
  }

  static async getActionStatusDevice({ uav_id, type }) {
    const { name, category } = await devicesController.getDevice(uav_id);
    // no type → every action of the device (prefix mode, no config lookup)
    if (!type) return actionRegistry.getActionStatus({ name });
    const actionDef = resolveCategoryConfig(category, 'actions', type);
    if (!actionDef) throw new Error(`Action '${type}' not configured for device ${name}`);
    return actionRegistry.getActionStatus({ actionServerName: buildDeviceName(name, actionDef) });
  }

  // Status for every action registered to a device, by name (no config lookup).
  static getActionStatusByName(name) {
    return actionRegistry.getActionStatus({ name });
  }

  static async cancelActionDevice({ uav_id, type }) {
    const { name, category } = await devicesController.getDevice(uav_id);
    const actionDef = resolveCategoryConfig(category, 'actions', type);
    if (!actionDef) throw new Error(`Action '${type}' not configured for device ${name}`);
    return actionRegistry.cancelAction({ actionServerName: buildDeviceName(name, actionDef) });
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
