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

describe('rosGateway: services > actions > publishers', () => {
  const calls = [];
  const controller = {
    callServiceDevice: async (a) => (calls.push(['service', a]), { state: 'success' }),
    sendActionGoalDevice: async (a) => (calls.push(['action', a]), { state: 'success' }),
    publishTopicDevice: async (a) => (calls.push(['publisher', a]), { state: 'success' }),
  };
  const gatewayFor = (category) => createRosGateway({ controller, categories: { getCategory: () => category } });
  const send = (gw) => gw.sendCommand({ device: DEVICE, type: 'Gimbal', attributes: { pitch: 1 } });

  test('a type declared as service goes to the service even if it is also a publisher', async () => {
    calls.length = 0;
    await send(gatewayFor({ services: { Gimbal: {} }, publishers: { Gimbal: {} } }));
    assert.equal(calls[0][0], 'service');
    assert.deepEqual(calls[0][1], { deviceId: 7, type: 'Gimbal', request: { pitch: 1 } });
  });

  test('action beats publisher', async () => {
    calls.length = 0;
    await send(gatewayFor({ actions: { Gimbal: {} }, publishers: { Gimbal: {} } }));
    assert.equal(calls[0][0], 'action');
  });

  test('publisher when it is the only declaration, with an empty message by default', async () => {
    calls.length = 0;
    await gatewayFor({ publishers: { Gimbal: {} } }).sendCommand({ device: DEVICE, type: 'Gimbal' });
    assert.deepEqual(calls[0], ['publisher', { deviceId: 7, type: 'Gimbal', message: {} }]);
  });

  test('an error thrown by the controller becomes an error response', async () => {
    const boom = createRosGateway({
      controller: { callServiceDevice: async () => { throw new Error('ROS not connected'); } },
      categories: { getCategory: () => ({ services: { Gimbal: {} } }) },
    });
    assert.deepEqual(await send(boom), { state: 'error', msg: 'ROS not connected' });
  });
});

describe('flatbuffer and mqtt gateways', () => {
  test('forward uav_id/type/attributes to their server', async () => {
    const seen = [];
    const server = { sendCommand: async (a) => (seen.push(a), { state: 'success', msg: 'sent' }) };
    for (const make of [createFlatbufferGateway, createMqttGateway]) {
      seen.length = 0;
      const r = await make({ getServer: () => server }).sendCommand({ device: DEVICE, type: 'commandMission', attributes: { data: true } });
      assert.deepEqual(seen, [{ uav_id: 7, type: 'commandMission', attributes: { data: true } }]);
      assert.equal(r.state, 'success');
    }
  });

  test('a disabled transport answers an error instead of throwing', async () => {
    for (const make of [createFlatbufferGateway, createMqttGateway]) {
      const r = await make({ getServer: () => null }).sendCommand({ device: DEVICE, type: 'x' });
      assert.equal(r.state, 'error');
      assert.match(r.msg, /disabled/);
    }
  });
});
