import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { EventEmitter } from 'node:events';
import { MqttServer } from '../models/mqtt/MqttServer.js';
import { parseTopic, requestTopic } from '../models/mqtt/mqttTopics.js';

// MqttServer against a fake client: no broker. The fake records what is published and lets
// a test play the drone by feeding `message` events back in.

class FakeClient extends EventEmitter {
  connected = true;
  published = [];
  subscribed = [];
  publish(topic, payload, opts, cb) {
    this.published.push({ topic, payload: JSON.parse(payload), opts });
    cb?.();
  }
  subscribe(topics, opts, cb) {
    this.subscribed.push(topics);
    cb?.();
  }
  end() {}
}

const DEVICE = { id: 7, name: 'uav1', category: 'cat' };
const CATEGORY = {
  subscribers: { position: { name: '/gps', messageType: 'sensor_msgs/NavSatFix' } },
  services: { command_mission: { name: '/start', messageType: 'std_srvs/SetBool' } },
};

function setup({ timeoutMs = 50, category = CATEGORY } = {}) {
  const client = new FakeClient();
  const updates = [];
  const server = new MqttServer('mqtt://test', {
    connect: () => client,
    timeoutMs,
    deps: {
      devices: { getDevice: async () => DEVICE, getByName: async (n) => (n === 'uav1' ? DEVICE : null) },
      positions: { updatePosition: (p) => updates.push(p) },
      categories: { getCategory: () => category },
    },
  });
  const droneSays = (topic, body) =>
    client.emit('message', topic, Buffer.from(typeof body === 'string' ? body : JSON.stringify(body)));
  return { client, server, updates, droneSays };
}

const call = (server) => server.sendCommand({ uav_id: 7, type: 'command_mission', attributes: { data: true } });
const tick = () => new Promise((r) => setImmediate(r));

describe('mqttTopics', () => {
  test('parses the three inbound shapes and rejects the rest', () => {
    assert.deepEqual(parseTopic('muav/uav1/pub/position'), { kind: 'pub', device: 'uav1', slot: 'position' });
    assert.deepEqual(parseTopic('muav/uav1/srv/command_mission/response'), {
      kind: 'response',
      device: 'uav1',
      name: 'command_mission',
    });
    assert.deepEqual(parseTopic('muav/uav1/status'), { kind: 'status', device: 'uav1' });
    assert.equal(parseTopic('muav/uav1/srv/command_mission/request'), null);
    assert.equal(parseTopic('other/uav1/status'), null);
  });

  test('builds the request topic', () => {
    assert.equal(requestTopic('uav1', 'command_mission'), 'muav/uav1/srv/command_mission/request');
  });
});

describe('MqttServer: connection', () => {
  test('subscribes on every connect (clean session)', () => {
    const { client } = setup();
    client.emit('connect');
    client.emit('connect');
    assert.equal(client.subscribed.length, 2);
    assert.ok(client.subscribed[0].includes('muav/+/pub/+'));
  });

  test('presence is tracked from the retained status topic', async () => {
    const { server, droneSays } = setup();
    droneSays('muav/uav1/status', 'online');
    await tick();
    assert.equal(server.getPresence('uav1'), 'online');
    assert.equal(server.getPresence('ghost'), 'unknown');
  });
});

describe('MqttServer: telemetry', () => {
  test('decodes a pub message into a position update', async () => {
    const { updates, droneSays } = setup();
    droneSays('muav/uav1/pub/position', { header: { seq: 3 }, latitude: 37.1, longitude: -5.9, altitude: 10 });
    await tick();
    assert.equal(updates.length, 1);
    assert.equal(updates[0].deviceId, 7);
    assert.equal(updates[0].latitude, 37.1);
  });

  test('NaN in a payload does not drop the message', async () => {
    const { updates, droneSays } = setup();
    droneSays('muav/uav1/pub/position', '{"header":{"seq":1},"latitude":37.1,"longitude":-5.9,"altitude":NaN}');
    await tick();
    assert.equal(updates.length, 1);
    assert.equal(updates[0].latitude, 37.1);
  });

  test('unknown device, unknown slot and garbage are ignored', async () => {
    const { updates, droneSays } = setup();
    droneSays('muav/ghost/pub/position', { latitude: 1 });
    droneSays('muav/uav1/pub/nothing', { latitude: 1 });
    droneSays('muav/uav1/pub/position', 'not json');
    await tick();
    assert.equal(updates.length, 0);
  });
});

describe('MqttServer: service calls', () => {
  test('publishes an enveloped request, QoS 1, not retained', async () => {
    const { client, server } = setup();
    const p = call(server);
    await tick();
    const [sent] = client.published;
    assert.equal(sent.topic, 'muav/uav1/srv/command_mission/request');
    assert.equal(sent.opts.qos, 1);
    assert.equal(sent.opts.retain, false);
    assert.deepEqual(sent.payload.request, { data: true });
    assert.equal(typeof sent.payload.id, 'string');
    assert.equal(typeof sent.payload.ts, 'number');
    client.emit('message', 'muav/uav1/srv/command_mission/response', Buffer.from(JSON.stringify({ id: sent.payload.id, success: true, response: { success: true } })));
    assert.equal((await p).state, 'success');
  });

  test('the service verdict inside `response` decides success', async () => {
    const { client, server, droneSays } = setup();
    const p = call(server);
    await tick();
    droneSays('muav/uav1/srv/command_mission/response', {
      id: client.published[0].payload.id,
      success: true,
      response: { success: false },
    });
    assert.equal((await p).state, 'error');
  });

  test('envelope success=false reports the drone-side error', async () => {
    const { client, server, droneSays } = setup();
    const p = call(server);
    await tick();
    droneSays('muav/uav1/srv/command_mission/response', {
      id: client.published[0].payload.id,
      success: false,
      error: 'service not available',
    });
    const r = await p;
    assert.equal(r.state, 'error');
    assert.match(r.msg, /service not available/);
  });

  test('concurrent calls of the same type are correlated by id', async () => {
    const { client, server, droneSays } = setup();
    const a = call(server);
    const b = call(server);
    await tick();
    const [first, second] = client.published.map((m) => m.payload.id);
    droneSays('muav/uav1/srv/command_mission/response', { id: second, success: true, response: { success: false } });
    droneSays('muav/uav1/srv/command_mission/response', { id: first, success: true, response: { success: true } });
    assert.equal((await a).state, 'success');
    assert.equal((await b).state, 'error');
  });

  test('times out and clears the pending call', async () => {
    const { server } = setup({ timeoutMs: 20 });
    const r = await call(server);
    assert.equal(r.state, 'error');
    assert.match(r.msg, /timeout/);
    assert.equal(server.pending.size, 0);
  });

  test('a late response after the timeout is ignored', async () => {
    const { client, server, droneSays } = setup({ timeoutMs: 20 });
    await call(server);
    droneSays('muav/uav1/srv/command_mission/response', {
      id: client.published[0].payload.id,
      success: true,
      response: { success: true },
    });
    assert.equal(server.pending.size, 0);
  });

  test('a service the category does not declare is a warning and sends nothing', async () => {
    const { client, server } = setup({ category: { services: {} } });
    const r = await call(server);
    assert.equal(r.state, 'warning');
    assert.equal(client.published.length, 0);
  });

  test('fails fast when the broker is down', async () => {
    const { client, server } = setup();
    client.connected = false;
    const r = await call(server);
    assert.equal(r.state, 'error');
    assert.equal(client.published.length, 0);
  });

  test('losing the broker connection fails pending calls instead of waiting for the timeout', async () => {
    const { client, server } = setup({ timeoutMs: 5000 });
    const p = call(server);
    await tick();
    client.emit('close');
    const r = await p;
    assert.equal(r.state, 'error');
    assert.equal(server.pending.size, 0);
  });
});
