import * as ROSLIB from 'roslib';
import { readDataFile } from '../../common/utils.js';
import { positionsController } from '../../controllers/positions.js';
import { decodeRosMsg } from './rosDecode.js';
import { categoryController } from '../../controllers/category.js';
import logger, { logHelpers } from '../../common/logger.js';

const devices_msg = readDataFile('../config/devices/devices_msg.yaml');
const uav_list = {};

export function RosSubscribe(uav_id, uav_type, type, msgType, callback, ros) {
  uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
    positionsController.updatePosition(callback({ msg, deviceId: uav_id, uav_type, type, msgType }));
  });
}

export function RosSubscribeCamera(uav_id, uav_type, type, msgType, callback, ros) {
  uav_list[uav_id]['listener_' + type].subscribe(function (msg) {
    positionsController.updateCamera(callback({ msg, deviceId: uav_id, uav_type, type, msgType }));
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
  // create listener
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
