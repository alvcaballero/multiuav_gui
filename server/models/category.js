import { readDataFile, writeDataFile } from '../common/utils.js';
import { devicesMsg, messagesTypes } from '../config/config.js';

const devices_msg = readDataFile(devicesMsg);
const messages_types = readDataFile(messagesTypes);
export class categoryModel {
  static getAll() {
    console.log('devices type');
    return Object.keys(devices_msg);
  }
  static getCategory(type) {
    console.log('devices category ' + type);
    if (devices_msg.hasOwnProperty(type)) {
      return devices_msg[type];
    }
    return devices_msg[type];
  }
  static updateCategory(type, value) {
    console.log('devices update ' + type);
    if (devices_msg.hasOwnProperty(type)) {
      devices_msg[type] = value;
      writeDataFile(devicesMsg, devices_msg);
    }
    return devices_msg[type];
  }
  static createCategory(type, value) {
    console.log('devices create ' + value);
    if (!devices_msg.hasOwnProperty(value)) {
      devices_msg[type] = value;
      writeDataFile(devicesMsg, devices_msg);
      return devices_msg[type];
    }
    return null;
  }
  static deleteCategory(type) {
    console.log('devices delete ' + type);
    if (devices_msg.hasOwnProperty(type)) {
      delete devices_msg[type];
      writeDataFile(devicesMsg, devices_msg);
    }
    return devices_msg[type];
  }

  static getMessagesType() {
    return messages_types;
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
    console.log('devices acction in category ' + type);
    if( devices_msg[type]?.attributes?.mission_action === undefined ) {
      return [];
    }
    return Object.values(devices_msg[type]['attributes']['mission_action']);
  }
}
