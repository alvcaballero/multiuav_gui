import { readDataFile, writeDataFile } from '../common/utils.js';
import { devicesMsg, missionSchema, messagesTypes } from '../config/config.js';
import logger from '../common/logger.js';

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
    logger.debug('categoryModel.getAll');
    return Object.keys(devices_msg);
  }
  static getCategory(type) {
    logger.debug(`categoryModel.getCategory: ${type}`);
    if (devices_msg.hasOwnProperty(type)) {
      return devices_msg[type];
    }
    return devices_msg[type];
  }
  static updateCategory(type, value) {
    logger.info(`categoryModel.updateCategory: ${type}`);
    if (devices_msg.hasOwnProperty(type)) {
      devices_msg[type] = value;
      writeDataFile(devicesMsg, devices_msg);
    }
    return devices_msg[type];
  }
  static createCategory(type, value) {
    logger.info(`categoryModel.createCategory: ${type}`);
    if (!devices_msg.hasOwnProperty(value)) {
      devices_msg[type] = value;
      writeDataFile(devicesMsg, devices_msg);
      return devices_msg[type];
    }
    return null;
  }
  static deleteCategory(type) {
    logger.info(`categoryModel.deleteCategory: ${type}`);
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
    logger.debug(`categoryModel.getAtributes: ${type}`);
    const attributes = getMissionAttributes(type);
    if (!attributes) return [];
    return Object.values(attributes.mission_param);
  }

  static getAttributesList(type) {
    logger.debug(`categoryModel.getAttributesList: ${type}`);
    const attributes = getMissionAttributes(type);
    if (!attributes) return [];
    return Object.values(attributes.mission_param).map(({ Name, id, type: fieldType, default: defaultValue }) => ({
      id,
      name: Name,
      type: fieldType,
      default: defaultValue ?? null,
    }));
  }

  static getAttributesDefaults(type) {
    logger.debug(`categoryModel.getAttributesDefaults: ${type}`);
    const attributes = getMissionAttributes(type);
    if (!attributes) return {};
    return Object.values(attributes.mission_param).reduce((acc, { id, default: defaultValue }) => {
      if (defaultValue !== undefined) acc[id] = defaultValue;
      return acc;
    }, {});
  }

  static getAtributesParam({ type, param }) {
    logger.debug(`categoryModel.getAtributesParam: ${type}-${param}`);
    const attributes = getMissionAttributes(type);
    if (!attributes) return {};
    return attributes.mission_param[param]?.param ?? {};
  }

  static getActions({ type }) {
    logger.debug(`categoryModel.getActions: ${type}`);
    const attributes = getMissionAttributes(type);
    if (!attributes) return [];
    return Object.values(attributes.mission_action);
  }
}
