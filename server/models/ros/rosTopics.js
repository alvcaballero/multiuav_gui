import * as ROSLIB from 'roslib';
import { readDataFile } from '../../common/utils.js';
import { encodeRosSrv } from './rosEncode.js';
import { buildTypeMap, validateRosMsg } from './rosValidateMSG.js';
import { getTopics, getMessageDetails, getPublishers } from './rosInspect.js';
import { logger, logHelpers } from '../../common/logger.js';

const devices_msg = readDataFile('../config/devices/devices_msg.yaml');

// Registry of active ROS topic subscriptions, indexed by device id.
// Each entry is a map of topicKey -> ROSLIB.Topic, kept only so the
// listeners can be unsubscribed later. Device metadata is NOT stored here.
// activeSubscriptions[deviceId] = { position: ROSLIB.Topic, camera: ROSLIB.Topic, ... }
const activeSubscriptions = {};

// Subscribe a device topic and forward each message to onMessage. The
// caller (the facade) supplies the effect callback, so position and camera
// topics share the same subscription primitive — they only differ in which
// onMessage they pass.
export function RosSubscribe({ uav_id, uav_type, type, msgName, msgType, onMessage }, ros) {
  // create listeners
  activeSubscriptions[uav_id][type] = new ROSLIB.Topic({
    ros: ros,
    name: msgName,
    messageType: msgType,
  });
  // subscribe devices
  activeSubscriptions[uav_id][type].subscribe(function (msg) {
    onMessage({ msg, deviceId: uav_id, uav_type, type, msgType });
  });
}

export async function subscribeDevice(uavAdded, ros, rosState, { onPosition, onCamera }) {
  if (!rosState || rosState.state != 'connect') {
    return { state: 'error', msg: 'ROS no conectado' };
  }
  logHelpers.ros.subscribe(uavAdded.id, uavAdded.name, { category: uavAdded.category });

  const { id, name, category, camera } = uavAdded;
  // Unsubscribe any stale listeners before re-subscribing so a reconnect
  // can't leave two live subscriptions feeding duplicate telemetry.
  if (activeSubscriptions[id]) {
    unsubscribeDevice(id);
  }
  activeSubscriptions[id] = {};
  let msgType = devices_msg[category]['subscribers'];

  // Only subscribe the camera topic when the device has a Websocket camera AND
  // the category config defines its message type — otherwise skip it below.
  const hasWebsocketCamera = camera.some((cam) => cam.type == 'Websocket');
  const cameraMsgTypeDefined = Boolean(msgType['camera']?.['messageType']);
  const shouldSubscribeCamera = hasWebsocketCamera && cameraMsgTypeDefined;

  if (hasWebsocketCamera && !cameraMsgTypeDefined) {
    logger.warn(`No message type found for camera subscription for ${name}`);
  }

  // subscribe devices
  Object.keys(devices_msg[category]['subscribers']).forEach((type) => {
    if (type == 'camera' && !shouldSubscribeCamera) {
      logger.debug(`Skipping camera subscription for ${name} as no websocket camera is present`);
      return;
    }
    RosSubscribe(
      {
        uav_id: id,
        uav_type: category,
        type: type,
        msgName: name + msgType[type]['name'],
        msgType: msgType[type]['messageType'],
        onMessage: type == 'camera' ? onCamera : onPosition,
      },
      ros
    );
  });
}

// Unsubscribe one device's listeners and drop its registry entry.
function unsubscribeOne(deviceId) {
  const listeners = activeSubscriptions[deviceId];
  if (!listeners) return;
  for (const topic of Object.values(listeners)) {
    topic.unsubscribe();
  }
  delete activeSubscriptions[deviceId];
}

// Unsubscribe a single device by id, or ALL devices when id < 0
// (id = -1 is used on every ROS disconnect to leave no orphaned subscriptions).
export async function unsubscribeDevice(id) {
  if (Object.keys(activeSubscriptions).length === 0) {
    return { state: 'success', msg: 'no quedan UAV de la lista' };
  }

  if (id < 0) {
    for (const deviceId of Object.keys(activeSubscriptions)) {
      unsubscribeOne(deviceId);
    }
    return { state: 'success', msg: 'Se han desuscrito todos los dispositivos' };
  }

  if (!activeSubscriptions[id]) {
    return { state: 'warning', msg: `device ${id} no estaba suscrito` };
  }
  unsubscribeOne(id);
  return { state: 'success', msg: `Se ha eliminado el dispositivo ${id}` };
}

export async function PubRosMsg(params, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  const { topic, messageType, message } = params;

  const msgStructure = await getMessageDetails(messageType, ros);
  const typeMap = buildTypeMap(msgStructure);
  validateRosMsg(messageType, message, typeMap);

  const subscribers = await getTopics(ros);
  if (!subscribers.topics.includes(topic)) {
    throw new Error(`No subscribers found for topic '${topic}'`);
  }

  const pub = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: messageType,
  });
  const rosMsg = encodeRosSrv({ type: '', msg: message, msgType: messageType });
  if (!rosMsg) {
    throw new Error('ROS message is empty or invalid');
  }

  pub.on('warning', function (warning) {
    logger.warn(`ROS publish warning: ${warning}`);
  });

  pub.publish(rosMsg);
  return { topic: topic, msgType: messageType, msg: 'Message published successfully' };
}

// Device layer — resolve the publisher from devices_msg config, then delegate to
// PubRosMsg (the primitive validates + encodes against the live rosbridge type).
// Mirror of rosServices.callService: publishers[type] gives the topic suffix and
// messageType; the final topic is `/${name}${suffix}`.
export async function publishTopic({ name, category, type, message }, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  const publishers = devices_msg[category]?.publishers;
  if (!publishers || !publishers.hasOwnProperty(type)) {
    return { state: 'warning', msg: `${type} to ${name} dont have this publisher` };
  }

  const messageType = publishers[type]['messageType'];
  const topic = `/${name}${publishers[type]['name']}`;
  try {
    await PubRosMsg({ topic, messageType, message }, ros);
    return { state: 'success', msg: `${type} to ${name} ok` };
  } catch (error) {
    const errMsg = error instanceof Error ? error.message : String(error);
    logger.error(`Error publishing topic: ${errMsg}`);
    return { state: 'error', msg: 'Failed to publish topic: ' + errMsg };
  }
}

export async function subscribeOnce({ topic, messageType, timeout = 2000 }, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  const topics = await getTopics(ros);
  if (!topics.topics.includes(topic)) {
    return Promise.reject(new Error(`Topic '${topic}' does not exist`));
  }

  const publisher = await getPublishers(topic, ros);
  if (publisher.length === 0) {
    logger.warn(
      `Warning: /rosapi/publishers returned empty list for topic '${topic}', but topic exists. Proceeding anyway...`
    );
  }

  const sub = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: messageType,
  });

  return new Promise((resolve, reject) => {
    const handler = (message) => {
      sub.unsubscribe();
      resolve(message);
    };

    sub.subscribe(handler);

    setTimeout(() => {
      sub.unsubscribe();
      reject(new Error('Timeout exceeded'));
    }, timeout);
  });
}
