import * as ROSLIB from 'roslib';
import { readDataFile } from '../../common/utils.js';
import { encodeRosSrv } from './rosEncode.js';
import { buildTypeMap, validateRosMsg } from './rosValidateMSG.js';
import { getTopics, getMessageDetails, getPublishers } from './rosInspect.js';
import logger, { logHelpers } from '../../common/logger.js';

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
export function RosSubscribe(uav_id, uav_type, type, msgType, onMessage) {
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
  let msgType = devices_msg[category]['topics'];
  // create listeners
  Object.keys(devices_msg[category]['topics']).forEach((element) => {
    activeSubscriptions[id][element] = new ROSLIB.Topic({
      ros: ros,
      name: name + devices_msg[category]['topics'][element]['name'],
      messageType: devices_msg[category]['topics'][element]['messageType'],
    });
  });
  // subscribe devices
  Object.keys(devices_msg[category]['topics']).forEach((element) => {
    if (element !== 'camera') {
      RosSubscribe(id, category, element, msgType[element]['messageType'], onPosition);
    }
  });
  // subscribe camera
  for (let i = 0; i < camera.length; i = i + 1) {
    logger.debug(`Camera type: ${camera[i]['type']}`);
    if (camera[i]['type'] == 'Websocket') {
      logger.debug(`camera websocket for ${name}`);
      RosSubscribe(id, category, 'camera', msgType['camera']['messageType'], onCamera);
    }
  }
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
