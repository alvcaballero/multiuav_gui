import { test, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { LINK_LEVEL } from '../models/radio/linkQuality.js';
import { levelOfPing, parsePingRtt, summarize } from '../models/radio/pingStats.js';
import { hostPingMonitor, pingOnce } from '../models/radio/hostPingMonitor.js';
import { validateDeviceAttributes } from '../schemas/zod/devices.js';

test('parsePingRtt: lee el tiempo de la salida de ping', () => {
  const out = '64 bytes from 10.20.0.51: icmp_seq=1 ttl=64 time=3.08 ms';
  assert.equal(parsePingRtt(out), 3.08);
  assert.equal(parsePingRtt('64 bytes from 10.20.0.51: icmp_seq=1 ttl=64 time<1 ms'), 1);
  assert.equal(parsePingRtt('1 packets transmitted, 0 received, 100% packet loss'), null);
  assert.equal(parsePingRtt(undefined), null);
});

test('summarize: media, jitter y pérdida sobre la ventana', () => {
  const s = summarize([10, 12, null, 10]);
  assert.equal(s.rttMs, 10);
  assert.equal(s.avgMs, 10.7);
  assert.equal(s.jitterMs, 2); // |12-10| and |10-12|
  assert.equal(s.lossPct, 25);
  assert.equal(s.samples, 4);
  assert.equal(summarize([]), null);
  assert.equal(summarize([5]).jitterMs, null);
});

test('levelOfPing: umbrales, una pérdida aislada y lost', () => {
  const ok = (n, rtt = 5) => Array(n).fill(rtt);
  assert.equal(levelOfPing([]), LINK_LEVEL.UNKNOWN);
  assert.equal(levelOfPing(ok(20)), LINK_LEVEL.GOOD);
  assert.equal(levelOfPing([...ok(19), null]), LINK_LEVEL.GOOD); // 5 % loss still good
  assert.equal(levelOfPing([...ok(17), null, null, 5]), LINK_LEVEL.FAIR); // 10 %
  assert.equal(levelOfPing(ok(20, 100)), LINK_LEVEL.FAIR); // slow but answering
  assert.equal(levelOfPing(ok(20, 400)), LINK_LEVEL.POOR);
  assert.equal(levelOfPing([...ok(10), ...Array(9).fill(null), 5]), LINK_LEVEL.POOR); // 45 % loss
  assert.equal(levelOfPing([...ok(5), null, null, null]), LINK_LEVEL.LOST);
  // the first pings after boot: not lost until LOST_AFTER_PINGS in a row
  assert.equal(levelOfPing([null]), LINK_LEVEL.UNKNOWN);
  assert.equal(levelOfPing([null, null, null]), LINK_LEVEL.LOST);
});

test('atributo companionIp: IPv4, null lo borra', () => {
  const dev = { protocol: 'ros', category: 'px4_ros2' };
  assert.equal(validateDeviceAttributes(dev, { companionIp: '10.20.0.51' }).success, true);
  assert.equal(validateDeviceAttributes(dev, { companionIp: null }).success, true);
  assert.equal(validateDeviceAttributes(dev, { companionIp: '10.20.0.51; rm -rf /' }).success, false);
  assert.equal(validateDeviceAttributes(dev, { companionIp: 'drone.local' }).success, false);
});

// --- monitor ---------------------------------------------------------------------------------------------
const device = (over = {}) => ({ id: 3, name: 'x500_1', attributes: { companionIp: '10.20.0.51' }, ...over });

let events;
let broadcasts;
eventBus.on(EVENTS.DEVICE_UPDATED, (devices) => broadcasts.push(devices));

beforeEach(() => {
  events = [];
  broadcasts = [];
  hostPingMonitor._tracks.clear();
  hostPingMonitor._addEvent = (e) => events.push(e);
});

const feed = (devices, samples, now = 1000) => {
  hostPingMonitor.track(devices);
  for (const rtt of samples) hostPingMonitor.record(devices[0].id, rtt, now);
  return hostPingMonitor.evaluate(devices, now);
};

test('un device sin companionIp no se sigue (statusOf → null)', () => {
  hostPingMonitor.track([device({ attributes: {} })]);
  assert.equal(hostPingMonitor.statusOf(3), null);
});

test('buen enlace: nivel, métricas y DEVICE_UPDATED con latency; sin Event', () => {
  assert.equal(feed([device()], [4, 6, 5]).length, 1);
  const status = hostPingMonitor.statusOf(3, 1500);
  assert.equal(status.level, LINK_LEVEL.GOOD);
  assert.equal(status.avgMs, 5);
  assert.equal(status.lossPct, 0);
  assert.equal(status.ageMs, 500);
  assert.equal(broadcasts[0][0].latency.level, LINK_LEVEL.GOOD);
  assert.equal(events.length, 0);
});

test('deja de contestar: lost con warning, y al volver info', () => {
  feed([device()], [5, 5, 5]);
  feed([device()], [null, null, null]);
  assert.equal(hostPingMonitor.statusOf(3).level, LINK_LEVEL.LOST);
  assert.deepEqual(
    events.map((e) => e.type),
    ['warning']
  );
  feed([device()], Array(20).fill(5)); // the window is replaced by good samples
  assert.equal(hostPingMonitor.statusOf(3).level, LINK_LEVEL.GOOD);
  assert.deepEqual(
    events.map((e) => e.type),
    ['warning', 'info']
  );
});

test('una computadora apagada al arrancar queda lost sin Event', () => {
  feed([device()], [null, null, null]);
  assert.equal(hostPingMonitor.statusOf(3).level, LINK_LEVEL.LOST);
  assert.equal(events.length, 0);
});

test('la ventana tiene tope y un cambio de dirección empieza de cero', () => {
  feed([device()], Array(50).fill(5));
  assert.equal(hostPingMonitor.statusOf(3).samples, 20);
  hostPingMonitor.track([device({ attributes: { companionIp: '10.20.0.52' } })]);
  assert.equal(hostPingMonitor.statusOf(3), null);
});

test('un device al que se le quita la dirección deja de seguirse', () => {
  feed([device()], [5]);
  hostPingMonitor.track([device({ attributes: {} })]);
  assert.equal(hostPingMonitor.statusOf(3), null);
});

// Real `ping`, on loopback: it only needs iputils on the host. Skipped where it is not installed.
test('pingOnce: contesta en loopback y devuelve null si no hay respuesta', async (t) => {
  let rtt;
  try {
    rtt = await pingOnce('127.0.0.1');
  } catch (err) {
    if (err.code === 'ENOENT') return t.skip('ping is not installed');
    throw err;
  }
  assert.equal(typeof rtt, 'number');
  assert.ok(rtt >= 0 && rtt < 100);
  assert.equal(await pingOnce('192.0.2.1'), null); // TEST-NET-1: nobody answers
});
