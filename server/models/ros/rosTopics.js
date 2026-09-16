import * as ROSLIB from 'roslib';
import { encodeRosSrv } from './rosEncode.js';
import { buildTypeMap, validateRosMsg } from './rosValidateMSG.js';
import { getTopics, getMessageDetails, getPublishers } from './rosInspect.js';
import { logger } from '../../common/logger.js';

// Registry of active ROS topic subscriptions, indexed by an opaque
// subscription key (a String — a device uses its id, an ad-hoc user
// subscription uses the topic name). Each entry is a map of
// slot -> ROSLIB.Topic, kept only so the listeners can be unsubscribed
// later. No device/category/topic-name knowledge lives here: the facade
// (ros.js) resolves all of that and hands over already-built topics.
// activeSubscriptions[key] = { position: ROSLIB.Topic, camera: ROSLIB.Topic, ... }
const activeSubscriptions = {};

// Subscribe a set of already-resolved topics under one key and forward each
// message to its per-topic onMessage. The caller (the facade) resolves names,
// message types and effect callbacks, so this primitive stays free of any
// devices_msg / category / camera business logic.
//   key    — opaque String grouping these subscriptions for later unsubscribe
//   topics — [{ slot, name, messageType, onMessage }]
export function subscribeTopics({ key, topics }, ros, rosState) {
  if (!rosState || rosState.state != 'connect') {
    return { state: 'error', msg: 'ROS no conectado' };
  }
  // Unsubscribe any stale listeners before re-subscribing so a reconnect
  // can't leave two live subscriptions feeding duplicate telemetry.
  if (activeSubscriptions[key]) {
    unsubscribeKey(key);
  }
  activeSubscriptions[key] = {};

  for (const { slot, name, messageType, onMessage } of topics) {
    const topic = new ROSLIB.Topic({ ros, name, messageType });
    topic.subscribe(function (msg) {
      onMessage({ msg, key, slot, messageType });
    });
    activeSubscriptions[key][slot] = topic;
  }
  return { state: 'success', msg: `Subscribed ${topics.length} topic(s) under ${key}` };
}

// Unsubscribe one key's listeners and drop its registry entry.
function unsubscribeOne(key) {
  const listeners = activeSubscriptions[key];
  if (!listeners) return;
  for (const topic of Object.values(listeners)) {
    topic.unsubscribe();
  }
  delete activeSubscriptions[key];
}

// Unsubscribe every registered key (used on ROS disconnect to leave no
// orphaned subscriptions).
export async function unsubscribeAll() {
  if (Object.keys(activeSubscriptions).length === 0) {
    return { state: 'success', msg: 'no quedan suscripciones activas' };
  }
  for (const key of Object.keys(activeSubscriptions)) {
    unsubscribeOne(key);
  }
  return { state: 'success', msg: 'Se han desuscrito todas las suscripciones' };
}

// Unsubscribe a single key. Keys are compared as strings, so callers passing a
// numeric device id and callers passing a topic name share one namespace.
export async function unsubscribeKey(key) {
  const strKey = String(key);
  if (!activeSubscriptions[strKey]) {
    return { state: 'warning', msg: `${strKey} no estaba suscrito` };
  }
  unsubscribeOne(strKey);
  return { state: 'success', msg: `Se ha eliminado la suscripción ${strKey}` };
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
