import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { readDataFile } from '../common/utils.js';
import { devicesMsg } from '../config/config.js';
import { categoryModel } from '../models/category.js';
import { decodableMsgTypes } from '../models/ros/rosDecode.js';

// GET /api/category/messages feeds the category editor. It is derived from the decoders and
// from what categories declare, so it can't drift the way the old messages.yaml did.
const devices = readDataFile(devicesMsg);
const TYPE_FIELD = { subscribers: 'messageType', publishers: 'messageType', services: 'serviceType', actions: 'actionType' };
const catalog = categoryModel.getMessagesType();

describe('category message catalog', () => {
  test('has the four devices_msg blocks', () => {
    assert.deepEqual(Object.keys(catalog).sort(), ['actions', 'publishers', 'services', 'subscribers']);
  });

  test('offers every endpoint some category declares, in its own block', () => {
    const missing = [];
    for (const [category, def] of Object.entries(devices)) {
      for (const [block, field] of Object.entries(TYPE_FIELD)) {
        for (const [key, entry] of Object.entries(def?.[block] ?? {})) {
          if (!catalog[block][key]?.includes(entry[field])) missing.push(`${category}.${block}.${key} ${entry[field]}`);
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
    assert.deepEqual(catalog.services.Gimbal, ['dji_osdk_ros/GimbalAction']);
    assert.deepEqual(catalog.publishers.Gimbal, ['psdk_interfaces/msg/GimbalRotation']);
  });
});
