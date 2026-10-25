import * as ROSLIB from 'roslib';
import { getTopics, getPublishers } from './rosInspect.js';
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
//   topics — [{ slot, name, messageType, onMessage, throttleMs? }] — throttleMs is rosbridge's own
//            throttle_rate: the gap it keeps between two messages, so the rest never cross the websocket
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

  for (const { slot, name, messageType, onMessage, throttleMs } of topics) {
    const topic = new ROSLIB.Topic({
      ros,
      name,
      messageType,
      ...(throttleMs > 0 && { throttle_rate: throttleMs, queue_length: 1 }),
    });
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
