import * as ROSLIB from 'roslib';
import { buildTypeMap, validateRosMsg } from './rosValidateMSG.js';
import { getTopics, getMessageDetails } from './rosInspect.js';
import { logger } from '../../common/logger.js';

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
