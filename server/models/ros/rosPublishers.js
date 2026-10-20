import * as ROSLIB from 'roslib';
import { buildTypeMap, validateRosMsg } from './rosValidateMSG.js';
import { getTopics, getMessageDetails } from './rosInspect.js';
import { logger } from '../../common/logger.js';

// Registry of active periodic publishers, indexed by an opaque key (a String —
// a device uses its id). Each entry keeps the advertised ROSLIB.Topic and its
// interval timer so both can be torn down later. Mirror of activeSubscriptions
// in rosSubscriptions.js. No device/category/topic-name knowledge lives here:
// the facade (ros.js) resolves all of that and hands over the built topic.
// activePublishers[key] = { topic: ROSLIB.Topic, timer: Timeout }
const activePublishers = {};

export async function PubRosMsgOnce(params, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  const { topic, messageType, message } = params;

  // `message` arrives already encoded for `messageType` (models/msgEncode.js: the gateway's
  // caller for commands, pubTopicOnce for the raw route); validate that encoded message.
  const rosMsg = message;
  if (!rosMsg) {
    throw new Error('ROS message is empty or invalid');
  }
  const msgStructure = await getMessageDetails(messageType, ros);
  const typeMap = buildTypeMap(msgStructure);
  validateRosMsg(messageType, rosMsg, typeMap);

  const subscribers = await getTopics(ros);
  if (!subscribers.topics.includes(topic)) {
    throw new Error(`No subscribers found for topic '${topic}'`);
  }

  const pub = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: messageType,
  });

  pub.on('warning', function (warning) {
    logger.warn(`ROS publish warning: ${warning}`);
  });

  pub.publish(rosMsg);
  return { topic: topic, msgType: messageType, msg: 'Message published successfully' };
}

// Publish a message at a constant rate under one key. The topic is advertised once and
// every tick only publishes, so (unlike PubRosMsgOnce) nothing is validated or looked up
// per message — and the topic need not have subscribers yet, which matters when the
// receiving side (e.g. the uXRCE-DDS agent) comes up after the GCS.
//   key          — opaque String grouping this publisher for later stop
//   buildMessage — () => message; called every tick so time-varying fields stay fresh
export function startPeriodic({ key, name, messageType, rateHz, buildMessage }, ros, rosState) {
  if (!rosState || rosState.state != 'connect') {
    return { state: 'error', msg: 'ROS no conectado' };
  }
  if (!(rateHz > 0)) {
    return { state: 'error', msg: `rate_hz inválido para ${name}: ${rateHz}` };
  }
  // Stop any stale publisher first so a reconnect can't leave two timers feeding the same topic.
  stopPeriodic(key);

  const topic = new ROSLIB.Topic({ ros, name, messageType });
  topic.advertise();
  const tick = () => {
    try {
      topic.publish(buildMessage());
    } catch (error) {
      logger.warn(`Periodic publish to ${name} failed: ${error.message}`);
    }
  };
  tick();
  activePublishers[String(key)] = { topic, timer: setInterval(tick, 1000 / rateHz) };
  return { state: 'success', msg: `Publishing ${name} at ${rateHz} Hz under ${key}` };
}

// Stop one key's publisher and drop its registry entry.
export function stopPeriodic(key) {
  const strKey = String(key);
  const entry = activePublishers[strKey];
  if (!entry) return { state: 'warning', msg: `${strKey} no estaba publicando` };
  clearInterval(entry.timer);
  entry.topic.unadvertise();
  delete activePublishers[strKey];
  return { state: 'success', msg: `Se ha detenido la publicación ${strKey}` };
}

// Stop every periodic publisher (used on ROS disconnect to leave no orphaned timers).
export function stopAllPeriodic() {
  for (const key of Object.keys(activePublishers)) {
    stopPeriodic(key);
  }
}
