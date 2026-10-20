import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { createCommandRegistry, endpointsOf } from '../models/commands/commandEncoders.js';

const PX4 = {
  publishers: { VehicleCommand: { name: '/fmu/in/vehicle_command', messageType: 'px4_msgs/msg/VehicleCommand' } },
  services: {
    commandMission: { name: '/offboard/start', messageType: 'std_srvs/srv/SetBool' },
    stopMission: { name: '/offboard/stop', messageType: 'std_srvs/srv/Trigger' },
  },
  subscribers: { battery: { name: '/battery', messageType: 'px4_msgs/msg/BatteryStatus' } },
};

const OTHER_FIRMWARE = {
  services: { takeoff: { name: '/cmd/takeoff', messageType: 'other_msgs/srv/Takeoff' } },
};

function registry() {
  const r = createCommandRegistry();
  r.register('Takeoff', { msgType: 'px4_msgs/msg/VehicleCommand' }, () => ({ command: 22 }));
  r.register('Takeoff', { msgType: 'other_msgs/srv/Takeoff' }, ({ attributes }) => ({ alt: attributes.altitude }));
  r.register('Land', { msgType: 'px4_msgs/msg/VehicleCommand' }, () => ({ command: 21 }));
  r.register('StopMission', { key: 'stopMission' }, () => ({}));
  r.register('CommandMission', { key: 'commandMission' }, () => ({ data: true }));
  r.registerLocal('SaveHome', () => 'home saved');
  return r;
}

describe('endpointsOf', () => {
  test('lists services, actions and publishers with their declared type, never subscribers', () => {
    const endpoints = endpointsOf(PX4);
    assert.deepEqual(endpoints.map((e) => `${e.block}.${e.key}=${e.msgType}`).sort(), [
      'publishers.VehicleCommand=px4_msgs/msg/VehicleCommand',
      'services.commandMission=std_srvs/srv/SetBool',
      'services.stopMission=std_srvs/srv/Trigger',
    ]);
  });
});

describe('register', () => {
  test('needs a key or a msgType', () => {
    assert.throws(() => createCommandRegistry().register('Land', {}, () => ({})), /needs a key or a msgType/);
  });

  test('rejects a duplicate encoder for the same command and endpoint', () => {
    const r = createCommandRegistry();
    r.register('Land', { msgType: 'a/B' }, () => ({}));
    assert.throws(() => r.register('Land', { msgType: 'a/B' }, () => ({})), /duplicate encoder/);
  });
});

describe('resolveCommands', () => {
  test('derives the commands from the endpoints the category already declares', () => {
    const { commands, problems } = registry().resolveCommands(PX4);
    assert.deepEqual(problems, []);
    assert.deepEqual([...commands.keys()].sort(), ['CommandMission', 'Land', 'SaveHome', 'StopMission', 'Takeoff']);
    assert.equal(commands.get('Takeoff').key, 'VehicleCommand');
    assert.equal(commands.get('Takeoff').block, 'publishers');
    assert.equal(commands.get('SaveHome'), null); // local: no endpoint
  });

  test('the same command resolves to another message on another firmware', () => {
    const { commands } = registry().resolveCommands(OTHER_FIRMWARE);
    assert.equal(commands.get('Takeoff').msgType, 'other_msgs/srv/Takeoff');
    assert.ok(!commands.has('Land'));
  });

  test('a generic msgType only implements the command its key names', () => {
    const { commands } = registry().resolveCommands({
      services: { pausemission: { messageType: 'std_srvs/srv/Trigger' } },
    });
    assert.ok(!commands.has('StopMission'));
  });

  test('a command that matches two endpoints is a problem until `via` picks one', () => {
    const dup = {
      ...PX4,
      services: { ...PX4.services, VehicleCommand: { messageType: 'px4_msgs/msg/VehicleCommand' } },
    };
    const ambiguous = registry().resolveCommands(dup);
    const takeoff = ambiguous.problems.find((p) => p.startsWith('Takeoff: matches'));
    assert.match(takeoff, /publishers\.VehicleCommand/);
    assert.match(takeoff, /services\.VehicleCommand/);
    assert.match(takeoff, /commands\.Takeoff\.via/);
    assert.ok(!ambiguous.commands.has('Takeoff'));

    const picked = registry().resolveCommands({
      ...dup,
      commands: { Takeoff: { via: 'publishers.VehicleCommand' }, Land: { via: 'publishers.VehicleCommand' } },
    });
    assert.deepEqual(picked.problems, []);
    assert.equal(picked.commands.get('Takeoff').block, 'publishers');
  });

  test('a `via` to a missing endpoint, or to one that does not implement the command, is a problem', () => {
    const r = registry();
    const missing = r.resolveCommands({ ...PX4, commands: { Takeoff: { via: 'services.nope' } } });
    assert.ok(missing.problems.some((p) => p.includes("via 'services.nope' is not an endpoint")));
    const wrong = r.resolveCommands({ ...PX4, commands: { Takeoff: { via: 'services.stopMission' } } });
    assert.ok(wrong.problems.some((p) => p.includes('has no encoder')));
  });

  test('a category without endpoints only has the local commands', () => {
    const { commands } = registry().resolveCommands(undefined);
    assert.deepEqual([...commands.keys()], ['SaveHome']);
  });
});

describe('encoderFor / encode', () => {
  test('(key, msgType) beats (key, ANY), which beats (ANY, msgType)', () => {
    const r = createCommandRegistry();
    r.register('Gimbal', { msgType: 'g/Msg' }, () => 'by msgType');
    r.register('Gimbal', { key: 'Gimbal' }, () => 'by key');
    r.register('Gimbal', { key: 'Gimbal', msgType: 'g/Msg' }, () => 'by both');
    assert.equal(r.encode('Gimbal', { block: 'services', key: 'Gimbal', msgType: 'g/Msg' }), 'by both');
    assert.equal(r.encode('Gimbal', { block: 'services', key: 'Gimbal', msgType: 'g/Other' }), 'by key');
    assert.equal(r.encode('Gimbal', { block: 'services', key: 'gimbal2', msgType: 'g/Msg' }), 'by msgType');
  });

  test('passes the attributes and the endpoint to the encoder', () => {
    const r = registry();
    const endpoint = { block: 'services', key: 'takeoff', msgType: 'other_msgs/srv/Takeoff' };
    assert.deepEqual(r.encode('Takeoff', endpoint, { attributes: { altitude: 12 } }), { alt: 12 });
  });

  test('encoding a command on an endpoint that does not implement it throws', () => {
    const endpoint = { block: 'services', key: 'stopMission', msgType: 'std_srvs/srv/Trigger' };
    assert.throws(() => registry().encode('Takeoff', endpoint), /no encoder for Takeoff/);
  });
});
