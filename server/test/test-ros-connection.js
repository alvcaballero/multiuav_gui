import { test } from 'node:test';
import assert from 'node:assert/strict';
import { WebSocketServer } from 'ws';

// config.js reads these at import time, so they are set before the module is loaded.
process.env.ROS_URL = 'ws://127.0.0.1:19091';
process.env.ROS_RECONNECT_INTERVAL_MS = '60';
const { RosConnection, serverStatus, getRos } = await import('../models/ros/rosConnection.js');

// The connection state belongs to an instance: nothing leaks between two of them.
test('cada RosConnection tiene su propio estado', () => {
  const a = new RosConnection();
  const b = new RosConnection();
  a.setState({ state: 'connect', msg: 'ok' });
  a.setVersionInfo({ version: 2, distro: 'humble' });
  assert.deepEqual(b.state, { state: 'disconnect', msg: 'init msg' });
  assert.deepEqual(b.versionInfo, { version: 1, distro: null });
  assert.deepEqual(a.versionInfo, { version: 2, distro: 'humble' });
});

test('una versión sin número se rechaza y no pisa la anterior', () => {
  const c = new RosConnection();
  assert.throws(() => c.setVersionInfo({ distro: 'x' }), /Invalid version/);
  assert.deepEqual(c.versionInfo, { version: 1, distro: null });
});

test('el timer de reconexión se arma una sola vez y se puede parar', () => {
  const c = new RosConnection();
  c._startReconnect();
  const timer = c._reconnectTimer;
  assert.ok(timer);
  c._startReconnect();
  assert.equal(c._reconnectTimer, timer, 'a second start must not stack another interval');
  c._stopReconnect();
  assert.equal(c._reconnectTimer, null);
});

test('un disconnect avisa al handler inyectado y arma la reconexión', () => {
  const c = new RosConnection();
  let called = 0;
  c.disconnect(() => called++);
  assert.equal(called, 1);
  assert.ok(c._reconnectTimer);
  c._stopReconnect();
});

test('la fachada por funciones sigue expuesta sobre la instancia compartida', () => {
  assert.equal(serverStatus().state, 'disconnect');
  assert.equal(getRos(), null);
});

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

test('onDisconnect corre una vez por conexión perdida, no en cada reintento fallido', async () => {
  let wss;
  const up = () => new Promise((resolve) => (wss = new WebSocketServer({ port: 19091 }, resolve)));
  const down = () => new Promise((resolve) => (wss.clients.forEach((c) => c.terminate()), wss.close(resolve)));
  const c = new RosConnection();
  const calls = { connected: 0, disconnected: 0 };
  try {
    // Never connected: attempts fail (error + close each) and must not call the handler.
    c.initAutoConnect(() => calls.connected++, () => calls.disconnected++);
    c.connect();
    await sleep(300);
    assert.equal(calls.disconnected, 0, 'a connection that never existed has nothing to undo');

    await up();
    await sleep(300);
    assert.equal(c.state.state, 'connect');
    assert.equal(calls.connected, 1);

    await down();
    await sleep(500); // several retries against a dead server
    assert.equal(c.state.state, 'disconnect');
    assert.equal(calls.disconnected, 1, 'error + close + retries are ONE lost connection');

    await up();
    await sleep(300);
    assert.equal(c.state.state, 'connect');
    assert.equal(calls.connected, 2);
    await down();
    await sleep(200);
    assert.equal(calls.disconnected, 2, 'a second connection lost notifies again');
  } finally {
    c._stopReconnect();
    c.ros?.close();
    await new Promise((resolve) => (wss ? wss.close(resolve) : resolve()));
  }
});

test('un intento reemplazado por un reintento no deja un socket huérfano abierto', async () => {
  // Handshake slower than the retry interval: every attempt is replaced while still connecting.
  const wss = await new Promise((resolve) => {
    const server = new WebSocketServer({ port: 19091, verifyClient: (_info, done) => setTimeout(() => done(true), 250) }, () => resolve(server));
  });
  const c = new RosConnection();
  try {
    c.initAutoConnect(() => {}, () => {});
    c.connect();
    await sleep(900);
    c._stopReconnect();
    c.ros?.close();
    await sleep(400); // let the last handshakes finish, so a leaked one would show as OPEN
    const open = [...wss.clients].filter((client) => client.readyState === 1).length;
    assert.equal(open, 0, `${open} orphan socket(s) left open on the server`);
  } finally {
    c._stopReconnect();
    wss.clients.forEach((client) => client.terminate());
    await new Promise((resolve) => wss.close(resolve));
  }
});
