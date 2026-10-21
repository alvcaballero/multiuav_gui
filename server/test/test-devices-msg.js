import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { categoryModel } from '../models/category.js';
import { decodeMsg, hasDecoder } from '../models/msgDecode.js';

// devices_msg.yaml is the SSOT for what each category publishes. These tests tie it to the
// decoder registry the way ros.js does at runtime: decoder picked by (slot, messageType),
// and the subscriber's own entry handed over as `params`.

const categories = () => categoryModel.getAll().map((name) => [name, categoryModel.getCategory(name)]);
const subscriber = (category, slot) => categoryModel.getCategory(category).subscribers[slot];
const decodeAs = (category, slot, msg) => {
  const entry = subscriber(category, slot);
  return decodeMsg({ msg, deviceId: 1, type: slot, msgType: entry.messageType, params: entry });
};

describe('devices_msg.yaml vs the decoder registry', () => {
  test('every subscriber declares a (slot, messageType) that has a decoder', () => {
    const undecoded = [];
    for (const [category, def] of categories()) {
      for (const [slot, entry] of Object.entries(def.subscribers ?? {})) {
        if (!hasDecoder(slot, entry.messageType)) undecoded.push(`${category}.${slot} (${entry.messageType})`);
      }
    }
    assert.deepEqual(undecoded, []);
  });

  // The message arrives as percentage 0-1: without `scale: 100` the cache rounds it to 0 or 1.
  test('categories whose battery arrives as 0-1 declare scale: 100 and report percent', () => {
    for (const category of ['px4_mavros', 'px4_mavros_slow', 'griffin', 'catec', 'fuvex', 'dji_M300_PSDK']) {
      assert.equal(subscriber(category, 'battery').scale, 100, `${category} battery scale`);
      const { batteryLevel } = decodeAs(category, 'battery', { percentage: 0.87 });
      assert.ok(Math.abs(batteryLevel - 87) < 1e-9, `${category}: ${batteryLevel}`);
    }
  });

  test('categories that already publish battery as percent declare no scale', () => {
    for (const category of ['dji_M210_noetic', 'dji_M300', 'dji_M600']) {
      assert.equal(subscriber(category, 'battery').scale, undefined, `${category} battery scale`);
    }
  });

  test('dji_M300_PSDK gimbal arrives in radians and is reported in degrees', () => {
    const { gimbal } = decodeAs('dji_M300_PSDK', 'gimbal', { vector: { x: Math.PI, y: 0, z: 0 } });
    assert.ok(Math.abs(gimbal.x - 180) < 1e-9);
  });

  test('sjtu heading is offset by -90, every other imu category is not', () => {
    const identity = { orientation: { w: 1, x: 0, y: 0, z: 0 } };
    assert.equal(decodeAs('sjtu', 'imu', identity).course, -90);
    assert.equal(decodeAs('dji_M300', 'imu', identity).course, 90);
  });

  test('sjtu sonar (sensor_msgs/Range) is decoded as a downward reading', () => {
    assert.deepEqual(decodeAs('sjtu', 'obstacle_info', { range: 2.5 }).obstacle_info, { down: 2.5 });
  });
});
