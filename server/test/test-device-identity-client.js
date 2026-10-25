import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import {
  capabilityEnabled,
  hasCapability,
  isDeviceIdentityValid,
  protocolsFor,
  withCapability,
  withCategory,
  withModel,
  withProtocol,
  vehicleTypesOf,
  VEHICLE_TYPE_LABELS,
} from '../../client/src/components/devices/deviceIdentity.js';

// The device form picks name → model → interface → protocol, and a change clears what depended on
// it. The client has no test runner: its pure logic is tested from here.

const PROTOCOLS = [
  { name: 'ros', enabled: true },
  { name: 'robofleet', enabled: true },
  { name: 'mqtt', enabled: false },
  { name: 'mavlink', enabled: true },
];
const x500OnMavlink = {
  name: 'uav_1',
  deviceModel: 'x500',
  category: 'px4_mavlink',
  protocol: 'mavlink',
  attributes: { systemId: 1 },
};

describe('device identity (client)', () => {
  test('a new model clears the interface, the protocol and its attributes: its interfaces are another list', () => {
    assert.deepEqual(withModel(x500OnMavlink, 'standard_vtol'), {
      name: 'uav_1',
      deviceModel: 'standard_vtol',
      category: null,
      protocol: null,
      attributes: {},
    });
    assert.equal(withModel(x500OnMavlink, 'x500'), x500OnMavlink); // the same: nothing changes
  });

  test('a new interface keeps the protocol only if it carries its message format', () => {
    const onRos = { ...x500OnMavlink, category: 'px4_ros2', protocol: 'ros', attributes: {} };
    assert.equal(withCategory(onRos, 'px4_mavros', 'ros_msg').protocol, 'ros');
    const toMavlink = withCategory(onRos, 'px4_mavlink', 'mavlink');
    assert.equal(toMavlink.protocol, null);
    assert.deepEqual(toMavlink.attributes, {});
  });

  test('a new protocol drops the attributes of the old one (mavlink systemId)', () => {
    assert.deepEqual(withProtocol(x500OnMavlink, 'ros').attributes, {});
    assert.deepEqual(withProtocol(x500OnMavlink, 'mavlink').attributes, { systemId: 1 });
  });

  test('only the protocols that carry the format are offered', () => {
    assert.deepEqual(protocolsFor(PROTOCOLS, 'mavlink').map((p) => p.name), ['mavlink']);
    assert.deepEqual(protocolsFor(PROTOCOLS, 'ros_msg').map((p) => p.name), ['ros', 'robofleet', 'mqtt']);
  });

  test('valid only with a model, an interface and a protocol (and a system id on mavlink)', () => {
    assert.ok(isDeviceIdentityValid(x500OnMavlink));
    assert.ok(!isDeviceIdentityValid({ ...x500OnMavlink, deviceModel: null }));
    assert.ok(!isDeviceIdentityValid({ ...x500OnMavlink, attributes: {} }));
  });
});

describe('device capabilities (client)', () => {
  const m210 = { deviceModel: 'dji_M210', category: 'dji_M210_noetic', protocol: 'ros', attributes: {} };

  test('a standard capability is on, an optional one off, until the unit switches it', () => {
    assert.equal(hasCapability(m210, 'gimbal', 'standard'), true);
    assert.equal(hasCapability(m210, 'rtk', 'optional'), false);
    const withRtk = withCapability(m210, 'rtk', 'optional', true);
    assert.deepEqual(withRtk.attributes, { capabilities: { rtk: true } });
    assert.equal(hasCapability(withRtk, 'rtk', 'optional'), true);
  });

  test('only what differs from the model is stored: switching back removes the entry', () => {
    const off = withCapability(m210, 'gimbal', 'standard', false);
    assert.deepEqual(off.attributes, { capabilities: { gimbal: false } });
    assert.deepEqual(withCapability(off, 'gimbal', 'standard', true).attributes, {});
  });

  test('a protocol change keeps the capability switches, a model change drops them', () => {
    const item = { ...x500OnMavlink, attributes: { systemId: 1, capabilities: { gimbal: true } } };
    assert.deepEqual(withProtocol(item, 'ros').attributes, { capabilities: { gimbal: true } });
    assert.deepEqual(withModel(item, 'standard_vtol').attributes, {});
  });

  // What decides if the form shows the stream sources (camera_stream) and the file sources (file_download).
  test('a capability is enabled by the model and the unit switch; a generic or unknown model shows everything', () => {
    const x500 = { capabilities: { file_download: 'standard', camera_stream: 'optional' } };
    assert.equal(capabilityEnabled(x500OnMavlink, x500, 'file_download'), true);
    assert.equal(capabilityEnabled(x500OnMavlink, x500, 'camera_stream'), false, 'optional: off until switched on');
    const withStream = withCapability(x500OnMavlink, 'camera_stream', 'optional', true);
    assert.equal(capabilityEnabled(withStream, x500, 'camera_stream'), true);
    assert.equal(capabilityEnabled(x500OnMavlink, { capabilities: {} }, 'file_download'), false, 'not declared');
    assert.equal(capabilityEnabled(x500OnMavlink, { capabilities: '*' }, 'camera_stream'), true);
    assert.equal(capabilityEnabled(x500OnMavlink, null, 'camera_stream'), true);
  });
});

describe('vehicle type filter (client)', () => {
  test('only the types that have models, in the form order; an unknown one goes last under its key', () => {
    const models = [{ vehicleType: 'ugv' }, { vehicleType: 'multirotor' }, { vehicleType: 'submarine' }, { vehicleType: 'multirotor' }];
    assert.deepEqual(vehicleTypesOf(models), ['multirotor', 'ugv', 'submarine']);
    assert.deepEqual(vehicleTypesOf(null), []);
  });
});

describe('VEHICLE_TYPE_LABELS mirrors the server', () => {
  test('every server vehicle type has its label, and no label names a type the server lacks', async () => {
    const { VEHICLE_TYPE } = await import('../config/status.js');
    assert.deepEqual(Object.keys(VEHICLE_TYPE_LABELS).sort(), Object.values(VEHICLE_TYPE).sort());
  });
});
