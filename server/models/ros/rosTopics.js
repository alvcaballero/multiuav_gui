import * as ROSLIB from 'roslib';
import { readDataFile } from '../../common/utils.js';
import { decodeRosMsg } from './rosDecode.js';
import { encodeRosSrv } from './rosEncode.js';
import { buildTypeMap, validateRosMsg } from './rosValidateMSG.js';
import { getTopics, getMessageDetails, getPublishers } from './rosInspect.js';
import logger, { logHelpers } from '../../common/logger.js';

const devices_msg = readDataFile('../config/devices/devices_msg.yaml');
const uav_list = {};

export function RosSubscribe(uav_id, uav_type, type, msgType, onMessage, ros) {
  uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
    onMessage({ msg, deviceId: uav_id, uav_type, type, msgType });
  });
}

export function RosSubscribeCamera(uav_id, uav_type, type, msgType, onMessage, ros) {
  uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
    onMessage({ msg, deviceId: uav_id, uav_type, type, msgType });
  });
}

export async function subscribeDevice(uavAdded, ros, rosState) {
  if (!rosState || rosState.state != 'connect') {
    return { state: 'error', msg: 'ROS no conectado' };
  }
  logHelpers.ros.subscribe(uavAdded.id, uavAdded.name, { category: uavAdded.category });

  const { id, name, category, camera } = uavAdded;
  uav_list[id] = uavAdded;
  let msgType = devices_msg[category]['topics'];
  // create listeners
  Object.keys(devices_msg[category]['topics']).forEach((element) => {
    uav_list[id]['listener_' + element] = new ROSLIB.Topic({
      ros: ros,
      name: name + devices_msg[category]['topics'][element]['name'],
      messageType: devices_msg[category]['topics'][element]['messageType'],
    });
  });
  // subscribe devices
  Object.keys(devices_msg[category]['topics']).forEach((element) => {
    if (element !== 'camera') {
      RosSubscribe(id, category, element, msgType[element]['messageType'], decodeRosMsg, ros);
    }
  });
  // subscribe camera
  for (let i = 0; i < camera.length; i = i + 1) {
    logger.debug(`Camera type: ${camera[i]['type']}`);
    if (camera[i]['type'] == 'Websocket') {
      logger.debug(`camera websocket for ${name}`);
      RosSubscribeCamera(id, category, 'camera', msgType['camera']['messageType'], decodeRosMsg, ros);
    }
  }
}

export async function unsubscribeDevice(id) {
  let cur_uav_idx;
  let Key_listener;
  if (Object.keys(uav_list).length != 0) {
    if (id < 0) {
      for (let i = 0; i < Object.keys(uav_list).length; i++) {
        cur_uav_idx = Object.values(uav_list).find((element) => element.id == id);
        if (cur_uav_idx) {
          Key_listener = cur_uav_idx.filter((element) => element.includes('listener'));
          Key_listener.forEach((element) => {
            uav_list[cur_uav_idx.id][element].unsubscribe();
          });
        }
      }
      var props = Object.getOwnPropertyNames(uav_list);
      for (var i = 0; i < props.length; i++) {
        delete uav_list[props[i]];
      }
    } else {
      cur_uav_idx = Object.values(uav_list).find((element) => element.id == id);

      Key_listener = Object.keys(cur_uav_idx).filter((element) => element.includes('listener'));

      if (Object.keys(uav_list).length != 0) {
        Key_listener.forEach((element) => {
          uav_list[cur_uav_idx.id][element].unsubscribe();
        });
        delete uav_list[cur_uav_idx.id];
        return { state: 'success', msg: 'Se ha eliminado el ' + cur_uav_idx };
      }
    }
  } else {
    return { state: 'success', msg: 'no quedan UAV de la lista' };
  }
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
