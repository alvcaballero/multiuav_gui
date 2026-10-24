import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { buildDeviceName, buildTelemetryName } from '../models/ros/rosNames.js';
import { validateDeviceAttributes } from '../schemas/zod/devices.js';

// The slow relay is a per-device ROS attribute: the companion republishes the telemetry at a lower
// rate under /<device>/slow<topic>. Only subscribers move.

describe('ROS names', () => {
  const position = { name: '/mavros/global_position/global' };

  test('a device topic is /<device><topic>', () => {
    assert.equal(buildDeviceName('uav_1', position), '/uav_1/mavros/global_position/global');
    assert.equal(buildTelemetryName('uav_1', position, {}), '/uav_1/mavros/global_position/global');
  });

  test('with the slow relay the telemetry is /<device>/slow<topic>', () => {
    assert.equal(buildTelemetryName('vtol_1', position, { slow: true }), '/vtol_1/slow/mavros/global_position/global');
    assert.equal(buildTelemetryName('vtol_1', position, { slow: false }), '/vtol_1/mavros/global_position/global');
  });
});

describe('the slow attribute', () => {
  test('is a ROS attribute: a mavlink or mqtt device may not carry it', () => {
    assert.ok(validateDeviceAttributes({ protocol: 'ros' }, { slow: true }).success);
    assert.ok(validateDeviceAttributes({}, { slow: true }).success); // no protocol = ros
    assert.ok(!validateDeviceAttributes({ protocol: 'mqtt' }, { slow: true }).success);
    assert.ok(!validateDeviceAttributes({ protocol: 'mavlink' }, { systemId: 1, slow: true }).success);
  });
});
