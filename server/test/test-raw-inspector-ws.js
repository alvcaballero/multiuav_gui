import { test, before, after } from 'node:test';
import assert from 'node:assert/strict';
import { createServer } from 'node:http';
import { setTimeout as sleep } from 'node:timers/promises';
import { WebSocket } from 'ws';
import { WebsocketManager } from '../WebsocketManager.js';
import { WebsocketInboundRouter } from '../WebsocketInboundRouter.js';
import { websocketController } from '../controllers/websocket.js';
import { WebSocketSubscriber } from '../subscribers/websocketSubscriber.js';
import { rawTap } from '../models/inspect/rawTap.js';
import { WS_RAW_INSPECT_INTERVAL_MS } from '../config/config.js';

// The inspector end to end, wired the way server.js wires it: a real WebSocket server, the real
// router, controller and subscriber, and real `ws` clients. Each piece has its own unit tests;
// this one is about the pieces being connected to each other.

let server;
let wsManager;
let controller;
let subscriber;
const clients = [];

// Opens a client and records everything it receives.
async function connect() {
  const { port } = server.address();
  const socket = new WebSocket(`ws://127.0.0.1:${port}/api/socket`);
  const client = { socket, received: [] };
  socket.on('message', (data) => client.received.push(JSON.parse(data.toString())));
  await new Promise((resolve, reject) => {
    socket.once('open', resolve);
    socket.once('error', reject);
  });
  clients.push(client);
  return client;
}

const send = (client, type, payload) => client.socket.send(JSON.stringify({ type, payload }));
const waitFor = async (predicate, ms = 1000) => {
  const until = Date.now() + ms;
  while (Date.now() < until) {
    if (predicate()) return true;
    await sleep(10);
  }
  return false;
};
const firstOf = (client, key) => client.received.find((m) => key in m)?.[key];
const publish = (over = {}) =>
  rawTap.publish({ deviceId: 1, slot: 'position', transport: 'ros', raw: { latitude: 37.5 }, ...over });

before(async () => {
  server = createServer();
  wsManager = new WebsocketManager(server, '/api/socket');
  controller = new websocketController(wsManager);
  wsManager.onClientConnect = null; // the welcome snapshot reads the database: not what is tested here
  const router = new WebsocketInboundRouter();
  wsManager.onMessage((client, raw) => router.handle(client, raw));
  wsManager.onClientClose = (clientId) => rawTap.removeClient(clientId);
  subscriber = new WebSocketSubscriber(controller);
  await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
});

after(async () => {
  clients.forEach(({ socket }) => socket.terminate());
  subscriber.cleanup();
  controller.destroy();
  wsManager.disconnect();
  await new Promise((resolve) => server.close(resolve));
});

test('subscribe is acknowledged, and the raw message reaches ONLY the client that asked', async () => {
  const a = await connect();
  const b = await connect();
  assert.notEqual(wsManager.clients.size, 0);

  send(a, 'raw:subscribe', { deviceId: 1, slot: 'position' });
  assert.ok(await waitFor(() => firstOf(a, 'rawStatus')), 'no ack');
  assert.deepEqual(firstOf(a, 'rawStatus'), { deviceId: 1, slot: 'position', ok: true });

  publish();
  assert.ok(await waitFor(() => firstOf(a, 'raw')), 'a did not receive the raw message');
  const { raw } = a.received.find((m) => m.raw);
  assert.equal(raw.deviceId, 1);
  assert.equal(raw.slot, 'position');
  assert.equal(raw.transport, 'ros');
  assert.deepEqual(raw.data, { latitude: 37.5 });

  await sleep(150);
  assert.deepEqual(b.received, [], 'b never asked: it must receive nothing');

  // targeted does not break broadcast: a normal message still reaches both
  controller.sendMessage({ server: { rosState: 'connect' } });
  assert.ok(await waitFor(() => firstOf(a, 'server') && firstOf(b, 'server')));
});

test('a refused subscribe says why, and nothing is delivered', async () => {
  const c = await connect();
  send(c, 'raw:subscribe', { deviceId: 1, slot: 'camera' });
  assert.ok(await waitFor(() => firstOf(c, 'rawStatus')));
  assert.deepEqual(firstOf(c, 'rawStatus'), { deviceId: 1, slot: 'camera', ok: false, reason: 'not_inspectable' });

  send(c, 'raw:subscribe', { slot: 'position' });
  assert.ok(await waitFor(() => c.received.filter((m) => m.rawStatus).length === 2));
  assert.equal(c.received.filter((m) => m.rawStatus)[1].rawStatus.reason, 'invalid');

  publish({ slot: 'camera' });
  await sleep(150);
  assert.equal(firstOf(c, 'raw'), undefined);
});

test('unsubscribe stops the stream', async () => {
  const d = await connect();
  send(d, 'raw:subscribe', { deviceId: 5, slot: 'battery' });
  assert.ok(await waitFor(() => firstOf(d, 'rawStatus')));
  send(d, 'raw:unsubscribe', { deviceId: 5, slot: 'battery' });
  await sleep(50);

  publish({ deviceId: 5, slot: 'battery', raw: { percentage: 80 } });
  await sleep(WS_RAW_INSPECT_INTERVAL_MS + 100);
  assert.equal(firstOf(d, 'raw'), undefined);
});

test('closing the socket removes the client and its subscriptions', async () => {
  const e = await connect();
  send(e, 'raw:subscribe', { deviceId: 9, slot: 'imu' });
  assert.ok(await waitFor(() => firstOf(e, 'rawStatus')));
  assert.ok(rawTap._subs.has('9:imu'));
  const before = wsManager.clients.size;

  e.socket.close();
  assert.ok(await waitFor(() => !rawTap._subs.has('9:imu')), 'the subscription outlived the socket');
  assert.ok(await waitFor(() => wsManager.clients.size === before - 1), 'the client outlived the socket');
});

test('a burst inside the window delivers the first and then the NEWEST message', async () => {
  const f = await connect();
  send(f, 'raw:subscribe', { deviceId: 2, slot: 'mission_state' });
  assert.ok(await waitFor(() => firstOf(f, 'rawStatus')));

  [1, 2, 3].forEach((n) => publish({ deviceId: 2, slot: 'mission_state', raw: { n } }));
  await waitFor(() => f.received.filter((m) => m.raw).length === 2, WS_RAW_INSPECT_INTERVAL_MS + 500);
  assert.deepEqual(
    f.received.filter((m) => m.raw).map((m) => m.raw.data.n),
    [1, 3],
  );
});
