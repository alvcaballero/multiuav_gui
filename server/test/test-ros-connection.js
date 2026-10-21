import { test } from 'node:test';
import assert from 'node:assert/strict';
import { RosConnection, serverStatus, getRos } from '../models/ros/rosConnection.js';

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
