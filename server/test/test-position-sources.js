import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { applyPatch, createDefaultPosition, POSITION_FRESH_MS } from '../models/positions/positionReducers.js';
import { decodeMsg } from '../models/msgDecode.js';

// Arbitration between the slots of a category that carry lat/lon (DJI: `position` GNSS and
// `position_rtk`). Pure: reducers + decoder, no database. The clock is injected (`now`).

const GNSS = { priority: 0, fresh_ms: 3000 };
const RTK = { priority: 10, fresh_ms: 3000 };
const fix = (lat, lon) => ({ header: { seq: 1 }, latitude: lat, longitude: lon, altitude: 12.34 });

// What ros.js does per message: decode with the slot's own entry, then merge.
const feed = (pos, slot, params, msg, now) => {
  const patch = decodeMsg({ msg, deviceId: 1, type: slot, msgType: 'sensor_msgs/NavSatFix', params });
  const effects = applyPatch(pos, patch, now);
  return { patch, effects };
};
const gnss = (pos, lat, lon, now) => feed(pos, 'position', GNSS, fix(lat, lon), now);
const rtk = (pos, lat, lon, now) => feed(pos, 'position_rtk', RTK, fix(lat, lon), now);

describe('position source arbitration', () => {
  test('a fresh RTK wins over GNSS', () => {
    const pos = createDefaultPosition(1);
    rtk(pos, 43.1, -8.1, 1000);
    gnss(pos, 43.2, -8.2, 1500);
    assert.equal(pos.latitude, 43.1);
    assert.equal(pos.attributes.positionSource, 'position_rtk');
  });

  test('a GNSS patch rejected by the arbitration loses its altitude and time too', () => {
    const pos = createDefaultPosition(1);
    rtk(pos, 43.1, -8.1, 1000);
    const { patch } = gnss(pos, 43.2, -8.2, 1500);
    assert.equal(patch.latitude, undefined);
    assert.equal(patch.altitude, undefined);
    assert.equal(pos.altitude, 12.3);
  });

  test('RTK that goes quiet hands the position back to GNSS, and wins again when it returns', () => {
    const pos = createDefaultPosition(1);
    rtk(pos, 43.1, -8.1, 1000);
    gnss(pos, 43.2, -8.2, 1500);
    assert.equal(pos.latitude, 43.1);

    gnss(pos, 43.2, -8.2, 1000 + 3000 + 1);
    assert.equal(pos.latitude, 43.2);
    assert.equal(pos.attributes.positionSource, 'position');

    const { effects } = rtk(pos, 43.1, -8.1, 6000);
    assert.equal(pos.latitude, 43.1);
    assert.equal(pos.attributes.positionSource, 'position_rtk');
    assert.equal(effects.length, 1);
    assert.equal(effects[0].type, 'info');
  });

  test('the source change is announced once, not on the first fix and not on every message', () => {
    const pos = createDefaultPosition(1);
    assert.equal(gnss(pos, 43.2, -8.2, 1000).effects.length, 0);
    assert.equal(gnss(pos, 43.2, -8.2, 1200).effects.length, 0);
    assert.equal(rtk(pos, 43.1, -8.1, 1300).effects.length, 1);
    assert.equal(rtk(pos, 43.1, -8.1, 1400).effects.length, 0);
  });

  test('lat = lon = 0 is not applied and does not make its source fresh', () => {
    const pos = createDefaultPosition(1);
    gnss(pos, 43.2, -8.2, 1000);
    rtk(pos, 0, 0, 1100);
    assert.equal(pos.latitude, 43.2);
    assert.equal(pos.attributes.positionSource, 'position');
    // the 0,0 RTK did not count as a sign of life: GNSS keeps flowing
    gnss(pos, 43.3, -8.3, 1200);
    assert.equal(pos.latitude, 43.3);
  });

  test('a lat = lon = 0 patch is dropped even without arbitration', () => {
    const pos = createDefaultPosition(1);
    applyPatch(pos, { deviceId: 1, latitude: 43, longitude: -8, altitude: 5, deviceTime: 't' });
    applyPatch(pos, { deviceId: 1, latitude: 0, longitude: 0, altitude: 0, deviceTime: 't2' });
    assert.equal(pos.latitude, 43);
    assert.equal(pos.altitude, 5);
  });

  test('the same priority never blocks: last message wins', () => {
    const pos = createDefaultPosition(1);
    feed(pos, 'position', { priority: 5 }, fix(1, 1), 1000);
    feed(pos, 'position_rtk', { priority: 5 }, fix(2, 2), 1100);
    feed(pos, 'position', { priority: 5 }, fix(3, 3), 1200);
    assert.equal(pos.latitude, 3);
  });

  test('without fresh_ms the default window applies', () => {
    const pos = createDefaultPosition(1);
    feed(pos, 'position_rtk', { priority: 10 }, fix(2, 2), 1000);
    feed(pos, 'position', { priority: 0 }, fix(3, 3), 1000 + POSITION_FRESH_MS);
    assert.equal(pos.latitude, 2);
    feed(pos, 'position', { priority: 0 }, fix(3, 3), 1000 + POSITION_FRESH_MS + 1);
    assert.equal(pos.latitude, 3);
  });

  test('a slot with no priority/fresh_ms behaves as before: applied, untagged, no source', () => {
    const pos = createDefaultPosition(1);
    const { patch, effects } = feed(pos, 'position', {}, fix(43.2, -8.2), 1000);
    assert.equal(patch.positionSource, undefined);
    assert.equal(pos.latitude, 43.2);
    assert.equal(pos.attributes.positionSource, undefined);
    assert.equal(effects.length, 0);
  });
});
