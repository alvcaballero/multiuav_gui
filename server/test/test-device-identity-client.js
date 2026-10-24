import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import {
  isDeviceIdentityValid,
  protocolsFor,
  withCategory,
  withModel,
  withProtocol,
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
