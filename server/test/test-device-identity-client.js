import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { validateDeviceAttributes } from '../schemas/zod/devices.js';
import {
  capabilityEnabled,
  companionIpError,
  hasCapability,
  isDeviceIdentityValid,
  protocolsFor,
  radioIpError,
  radioMacError,
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

describe('on-board hardware attributes (client)', () => {
  // The radio (radioIp / radioMac) and the onboard computer (companionIp) belong to the unit, not to the
  // protocol: changing the protocol or the model must not wipe the address the operator typed.
  const radio = { radioIp: '10.20.0.64', radioMac: 'ac:8b:a9:c0:a1:cb', companionIp: '10.20.0.51' };
  const onMavlink = { ...x500OnMavlink, attributes: { systemId: 1, ...radio } };

  test('a new protocol drops the protocol attributes but keeps the radio and the computer', () => {
    assert.deepEqual(withProtocol(onMavlink, 'ros').attributes, radio);
    assert.deepEqual(
      withProtocol(onMavlink, 'mavlink').attributes,
      { systemId: 1, ...radio },
      'same protocol: all kept'
    );
  });

  test('a new protocol keeps the capability switches next to the hardware', () => {
    const item = { ...onMavlink, attributes: { ...onMavlink.attributes, capabilities: { rtk: true } } };
    assert.deepEqual(withProtocol(item, 'ros').attributes, { ...radio, capabilities: { rtk: true } });
  });

  test('a new model drops the capability switches and the protocol attributes, not the hardware', () => {
    const item = { ...onMavlink, attributes: { ...onMavlink.attributes, capabilities: { rtk: true } } };
    const changed = withModel(item, 'standard_vtol');
    assert.deepEqual(changed.attributes, radio);
    assert.equal(changed.category, null);
    assert.equal(changed.protocol, null);
  });

  test('a new interface that cannot keep the protocol keeps the hardware too', () => {
    const onRos = { ...x500OnMavlink, category: 'px4_ros2', protocol: 'ros', attributes: { ...radio } };
    assert.deepEqual(withCategory(onRos, 'px4_mavlink', 'mavlink').attributes, radio);
  });

  test('a cleared field (null or empty) is not carried over', () => {
    const cleared = {
      ...onMavlink,
      attributes: { systemId: 1, radioIp: null, radioMac: '', companionIp: '10.20.0.51' },
    };
    assert.deepEqual(withProtocol(cleared, 'ros').attributes, { companionIp: '10.20.0.51' });
  });

  test('the identity is invalid with a malformed address, valid with none or a good one', () => {
    const base = { ...x500OnMavlink };
    for (const bad of [
      { radioIp: '10.20.0.300' },
      { radioIp: '10.20.0' },
      { radioMac: 'aa:bb:cc' },
      { radioMac: 'not a mac' },
      { companionIp: 'drone.local' },
      { companionIp: '10.20.0.51; rm -rf /' },
    ]) {
      assert.ok(!isDeviceIdentityValid({ ...base, attributes: { systemId: 1, ...bad } }), JSON.stringify(bad));
    }
    assert.ok(isDeviceIdentityValid({ ...base, attributes: { systemId: 1, ...radio } }));
    assert.ok(
      isDeviceIdentityValid({ ...base, attributes: { systemId: 1, radioIp: null, radioMac: '', companionIp: null } })
    );
    assert.ok(
      isDeviceIdentityValid({ ...base, attributes: { systemId: 1, radioMac: 'AC-8B-A9-C0-A1-CB' } }),
      'dashes, upper case'
    );
    assert.ok(
      isDeviceIdentityValid({ ...base, attributes: { systemId: 1, radioIp: ' 10.20.0.64 ' } }),
      'surrounding spaces'
    );
  });

  test('a bad address does not hide the missing system id, nor the other way round', () => {
    assert.ok(!isDeviceIdentityValid({ ...x500OnMavlink, attributes: { ...radio } }), 'mavlink without systemId');
    assert.ok(!isDeviceIdentityValid({ ...x500OnMavlink, attributes: { systemId: 1, radioIp: 'x' } }));
  });

  test('each error is a message to show under the field, null when fine', () => {
    assert.equal(radioIpError('10.20.0.64'), null);
    assert.equal(companionIpError(''), null);
    assert.equal(radioMacError(undefined), null);
    assert.match(radioIpError('10.20'), /IPv4/);
    assert.match(companionIpError('abc'), /IPv4/);
    assert.match(radioMacError('abc'), /MAC/);
  });
});

describe('the address validators agree with the server schema', () => {
  // The client only avoids sending what the server would refuse (client/CLAUDE.md › Device attributes): if the two
  // drift, the form either blocks a good address or lets a bad one through to a 400.
  const device = { protocol: 'ros', category: 'px4_ros2' };
  const server = (key, value) => validateDeviceAttributes(device, { [key]: value }).success;

  const IPS = [
    '10.20.0.64',
    '0.0.0.0',
    '255.255.255.255',
    '192.168.1.1',
    '10.20.0.300',
    '256.1.1.1',
    '10.20.0',
    '10.20.0.1.5',
    '01.2.3.4',
    '10.020.0.1',
    '1.2.3.-4',
    '1..2.3',
    'drone.local',
    '::1',
    ' 10.20.0.64 ',
    '10.20.0.64; ls',
    '',
  ];
  const MACS = [
    'ac:8b:a9:c0:a1:cb',
    'AC:8B:A9:C0:A1:CB',
    'ac-8b-a9-c0-a1-cb',
    'ac:8b:a9:c0:a1',
    'ac:8b:a9:c0:a1:cb:00',
    'ac8b.a9c0.a1cb',
    'gg:8b:a9:c0:a1:cb',
    'ac:8b-a9:c0-a1:cb',
    ' ac:8b:a9:c0:a1:cb ',
    '',
  ];

  for (const ip of IPS) {
    test(`IPv4 ${JSON.stringify(ip)}`, () => {
      const accepted = server('radioIp', ip === '' ? null : ip); // the form sends null for an empty field
      assert.equal(radioIpError(ip) === null, accepted, 'radioIp');
      assert.equal(companionIpError(ip) === null, server('companionIp', ip === '' ? null : ip), 'companionIp');
    });
  }

  for (const mac of MACS) {
    test(`MAC ${JSON.stringify(mac)}`, () => {
      assert.equal(radioMacError(mac) === null, server('radioMac', mac === '' ? null : mac));
    });
  }
});
