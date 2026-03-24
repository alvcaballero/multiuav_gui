import { readDataFile, writeDataFile } from '../common/utils.js';
import { devicesMsg, missionSchema, messagesTypes } from '../config/config.js';

const devices_msg = readDataFile(devicesMsg);
const messages_types = readDataFile(messagesTypes);
const _mission_schema = readDataFile(missionSchema);

const getMissionAttributes = (type) => {
  const schema = devices_msg[type]?.mission_schema;
  if (!schema) return undefined;
  const base = {
    mission_action: _mission_schema.mission_action,
    mission_param: { ..._mission_schema.mission_param },
  };
  if (schema === 'v2') {
    base.mission_param = { ...base.mission_param, ..._mission_schema.mission_param_v2 };
  }
  return base;
};
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
    const attributes = getMissionAttributes(type);
    if (!attributes) return [];
    return Object.values(attributes.mission_param);
  }
  static getAtributesParam({ type, param }) {
    console.log('devices atributes ' + type + '-' + param);
    const attributes = getMissionAttributes(type);
    if (!attributes) return {};
    return attributes.mission_param[param]?.param ?? {};
  }

  static getActions({ type }) {
    console.log('get device actions ' + type);
    const attributes = getMissionAttributes(type);
    if (!attributes) return [];
    return Object.values(attributes.mission_action);
  }
}
