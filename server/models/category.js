import { readJSON, readYAML } from '../common/utils.js';

const devices_msg = readYAML('../config/devices/devices_msg.yaml');

export class categoryModel {
  static getAll() {
    console.log('devices type');
    return Object.keys(devices_msg);
  }
  static getAtributes(type) {
    console.log('devices attributes ' + type);
    return Object.values(devices_msg[type]['attributes']['mission_param']);
  }
  static getAtributesParam({ type, param }) {
    console.log('devices atributes ' + type + '-' + param);
    let response = {};
    if (devices_msg.hasOwnProperty(type)) {
      console.log(devices_msg[type]['attributes']['mission_param'][param]['param']);
      response = devices_msg[type]['attributes']['mission_param'][param]['param'];
    }
    return response;
  }

  static getActions({ type }) {
    console.log('devices acction ' + type);
    return Object.values(devices_msg[type]['attributes']['mission_action']);
  }
}
