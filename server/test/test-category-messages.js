import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { readDataFile } from '../common/utils.js';
import { devicesMsg } from '../config/config.js';
import { categoryModel, findLegacyTypeFields } from '../models/category.js';
import { decodableMsgTypes } from '../models/ros/rosDecode.js';

// GET /api/category/messages feeds the category editor. It is derived from the decoders and
// from what categories declare, so it can't drift the way the old messages.yaml did.
const devices = readDataFile(devicesMsg);
const BLOCKS = ['subscribers', 'publishers', 'services', 'actions'];
const catalog = categoryModel.getMessagesType();

describe('category message catalog', () => {
  test('has the four devices_msg blocks', () => {
    assert.deepEqual(Object.keys(catalog).sort(), ['actions', 'publishers', 'services', 'subscribers']);
  });

  test('offers every endpoint some category declares, in its own block', () => {
    const missing = [];
    for (const [category, def] of Object.entries(devices)) {
      for (const block of BLOCKS) {
        for (const [key, entry] of Object.entries(def?.[block] ?? {})) {
          if (!catalog[block][key]?.includes(entry.messageType)) missing.push(`${category}.${block}.${key} ${entry.messageType}`);
        }
      }
    }
    assert.deepEqual(missing, []);
  });

  test('offers every type a subscriber slot can decode, even if no category uses it yet', () => {
    for (const [slot, types] of decodableMsgTypes()) {
      for (const msgType of types) assert.ok(catalog.subscribers[slot].includes(msgType), `${slot} ${msgType}`);
    }
  });

  test('a command key only lists the types categories declare for it', () => {
    assert.deepEqual(catalog.services.gimbal_control, ['dji_osdk_ros/GimbalAction']);
    assert.deepEqual(catalog.publishers.gimbal_control, ['psdk_interfaces/msg/GimbalRotation']);
  });
});

describe('devices_msg type field', () => {
  test('every block of the real file names its type with messageType', () => {
    assert.deepEqual(findLegacyTypeFields(devices), []);
  });

  test('serviceType/actionType are reported with their path', () => {
    const legacy = {
      some_drone: {
        services: { command_mission: { name: '/start', serviceType: 'std_srvs/SetBool' } },
        actions: { camera_file_download: { name: '/dl', actionType: 'x/action/Download' } },
        publishers: { gimbal_control: { name: '/g', messageType: 'x/msg/Gimbal' } },
      },
    };
    assert.deepEqual(findLegacyTypeFields(legacy), [
      'some_drone.services.command_mission.serviceType',
      'some_drone.actions.camera_file_download.actionType',
    ]);
  });
});
