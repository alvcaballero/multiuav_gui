import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { registerGateway, getGateway, registeredProtocols } from '../models/gateways/registry.js';
import '../models/gateways/index.js';
import { createRosGateway } from '../models/gateways/rosGateway.js';
import { createFlatbufferGateway } from '../models/gateways/flatbufferGateway.js';
import { createMqttGateway } from '../models/gateways/mqttGateway.js';
import { DEVICE_PROTOCOL } from '../schemas/zod/devices.js';

const DEVICE = { id: 7, name: 'uav1', category: 'cat' };

describe('gateway registry', () => {
  test('rejects a gateway without protocol or sendCommand', () => {
    assert.throws(() => registerGateway({ protocol: 'x' }));
    assert.throws(() => registerGateway({ sendCommand() {} }));
  });

  test('rejects a protocol registered twice', () => {
    assert.throws(() => registerGateway({ protocol: 'ros', sendCommand() {} }), /already registered/);
  });

  test('an unknown protocol has no gateway', () => {
    assert.equal(getGateway('carrier-pigeon'), undefined);
  });

  // The zod enum is static on purpose (schemas must not import models): this is the drift guard.
  test('DEVICE_PROTOCOL and the registered gateways name the same protocols', () => {
    assert.deepEqual([...registeredProtocols()].sort(), [...DEVICE_PROTOCOL.options].sort());
  });
});

describe('rosGateway: the resolved endpoint picks the transport', () => {
  const calls = [];
  const controller = {
    callServiceDevice: async (a) => (calls.push(['service', a]), { state: 'success' }),
    sendActionGoalDevice: async (a) => (calls.push(['action', a]), { state: 'success' }),
    publishTopicDevice: async (a) => (calls.push(['publisher', a]), { state: 'success' }),
  };
  const gateway = createRosGateway({ controller });
  const send = (block, message) =>
    gateway.sendCommand({ device: DEVICE, endpoint: { block, key: 'gimbal_control', msgType: 'x/Y' }, message });

  test('a services endpoint is called as a service, with the message as request', async () => {
    calls.length = 0;
    await send('services', { pitch: 1 });
    assert.deepEqual(calls[0], ['service', { deviceId: 7, type: 'gimbal_control', request: { pitch: 1 } }]);
  });

  test('an actions endpoint sends a goal', async () => {
    calls.length = 0;
    await send('actions', { pitch: 1 });
    assert.equal(calls[0][0], 'action');
  });

  test('a publishers endpoint publishes, with an empty message by default', async () => {
    calls.length = 0;
    await send('publishers');
    assert.deepEqual(calls[0], ['publisher', { deviceId: 7, type: 'gimbal_control', message: {} }]);
  });

  test('an error thrown by the controller becomes an error response', async () => {
    const boom = createRosGateway({
      controller: { callServiceDevice: async () => { throw new Error('ROS not connected'); } },
    });
    const r = await boom.sendCommand({ device: DEVICE, endpoint: { block: 'services', key: 'gimbal_control' }, message: {} });
    assert.deepEqual(r, { state: 'error', msg: 'ROS not connected' });
  });
});

describe('flatbuffer and mqtt gateways', () => {
  test('forward uav_id, the endpoint key and the message to their server', async () => {
    const seen = [];
    const server = { sendCommand: async (a) => (seen.push(a), { state: 'success', msg: 'sent' }) };
    const endpoint = { block: 'services', key: 'command_mission', msgType: 'std_srvs/SetBool' };
    for (const make of [createFlatbufferGateway, createMqttGateway]) {
      seen.length = 0;
      const r = await make({ getServer: () => server }).sendCommand({ device: DEVICE, endpoint, message: { data: true } });
      assert.deepEqual(seen, [{ uav_id: 7, type: 'command_mission', attributes: { data: true } }]);
      assert.equal(r.state, 'success');
    }
  });

  test('a disabled transport answers an error instead of throwing', async () => {
    for (const make of [createFlatbufferGateway, createMqttGateway]) {
      const r = await make({ getServer: () => null }).sendCommand({ device: DEVICE, endpoint: { key: 'x' } });
      assert.equal(r.state, 'error');
      assert.match(r.msg, /disabled/);
    }
  });
});
