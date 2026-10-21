import { test, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { telemetryWatchdog, SLOT_HEALTH, slotTimeoutMs } from '../models/inspect/telemetryWatchdog.js';
import { categoryModel } from '../models/category.js';
import { TELEMETRY_SLOT_TIMEOUT_MS } from '../config/config.js';

// px4_ros2: vehicle_command_ack is `watchdog: false`, camera needs a Websocket camera.
const device = (over = {}) => ({ id: 1, name: 'px4_1', category: 'px4_ros2', camera: [], ...over });
const WATCHED = ['position_global', 'local_position', 'battery', 'vehicle_status', 'gimbal', 'obstacle_info'];
const T = TELEMETRY_SLOT_TIMEOUT_MS;

const recordAll = (now, slots = WATCHED, id = 1) => slots.forEach((slot) => telemetryWatchdog.record(id, slot, now));

let events;
let broadcasts;
eventBus.on(EVENTS.DEVICE_UPDATED, (devices) => broadcasts.push(devices));

beforeEach(() => {
  events = [];
  broadcasts = [];
  telemetryWatchdog._devices.clear();
  telemetryWatchdog._addEvent = (e) => events.push(e);
});

test('todos los slots al día: sin problemas ni eventos', () => {
  recordAll(0);
  recordAll(T);
  assert.deepEqual(telemetryWatchdog.check([device()], T + 1), []);
  assert.deepEqual(telemetryWatchdog.statusOf(1), {});
  assert.equal(events.length, 0);
});

test('un slot que nunca llegó es missing recién después del período de gracia', () => {
  const others = WATCHED.filter((s) => s !== 'battery');
  recordAll(0, others);
  telemetryWatchdog.check([device()], T - 1);
  assert.deepEqual(telemetryWatchdog.statusOf(1), {});

  recordAll(T, others);
  telemetryWatchdog.check([device()], T + 1);
  assert.deepEqual(telemetryWatchdog.statusOf(1), { battery: SLOT_HEALTH.MISSING });
  assert.equal(events.length, 1);
  assert.equal(events[0].type, 'warning');
  assert.equal(events[0].attributes.slot, 'battery');
  assert.match(events[0].attributes.message, /battery_status_v1/); // names the topic: it is a config problem
});

test('un slot que deja de llegar es stale, avisa una sola vez y vuelve a ok con un info', () => {
  recordAll(0);
  const others = WATCHED.filter((s) => s !== 'gimbal');
  recordAll(T, others);
  telemetryWatchdog.check([device()], T + 1);
  recordAll(T + 2, others);
  telemetryWatchdog.check([device()], T + 3);
  assert.deepEqual(telemetryWatchdog.statusOf(1), { gimbal: SLOT_HEALTH.STALE });
  assert.equal(events.length, 1, 'transitions only, not every tick');

  telemetryWatchdog.record(1, 'gimbal', T + 4);
  telemetryWatchdog.check([device()], T + 5);
  assert.deepEqual(telemetryWatchdog.statusOf(1), {});
  assert.equal(events.length, 2);
  assert.equal(events[1].type, 'info');
});

test('no espera los slots con watchdog: false ni la cámara de un device sin cámara Websocket', () => {
  recordAll(0);
  recordAll(T);
  telemetryWatchdog.check([device()], T + 1);
  assert.equal(telemetryWatchdog.statusOf(1).vehicle_command_ack, undefined);
  assert.equal(telemetryWatchdog.statusOf(1).camera, undefined);

  telemetryWatchdog.check([device({ camera: [{ type: 'Websocket' }] })], T + 2);
  assert.deepEqual(telemetryWatchdog.statusOf(1), { camera: SLOT_HEALTH.MISSING });
});

test('un device que no manda nada no da warnings por slot: eso es OFFLINE', () => {
  recordAll(0);
  telemetryWatchdog.check([device()], T * 3);
  assert.deepEqual(telemetryWatchdog.statusOf(1), {});
  assert.equal(events.length, 0);
});

test('un device que ya no existe se olvida', () => {
  recordAll(0);
  telemetryWatchdog.check([], 1);
  assert.equal(telemetryWatchdog._devices.size, 0);
});

test('timeout_s de la entrada pisa el timeout por defecto', () => {
  const battery = categoryModel.getCategory('px4_ros2').subscribers.battery;
  battery.timeout_s = (T * 3) / 1000;
  try {
    recordAll(0);
    recordAll(T * 2, WATCHED.filter((s) => s !== 'battery'));
    telemetryWatchdog.check([device()], T * 2 + 1);
    assert.deepEqual(telemetryWatchdog.statusOf(1), {});
  } finally {
    delete battery.timeout_s;
  }
});

test('slotTimeoutMs: timeout_s gana, rate_hz da tres períodos, nunca bajo el default', () => {
  assert.equal(slotTimeoutMs({}), T);
  assert.equal(slotTimeoutMs(undefined), T);
  assert.equal(slotTimeoutMs({ rate_hz: 0.5 }), 6000); // 3 / 0.5 Hz
  assert.equal(slotTimeoutMs({ rate_hz: 4 }), T); // 0.75 s < default
  assert.equal(slotTimeoutMs({ rate_hz: 0.5, timeout_s: 20 }), 20000);
  assert.equal(slotTimeoutMs({ rate_hz: 0 }), T);
});

test('un slot lento con rate_hz no oscila a stale tras un par de pérdidas', () => {
  const battery = categoryModel.getCategory('px4_ros2').subscribers.battery;
  battery.rate_hz = 0.5; // one every 2 s → 6 s of tolerance
  try {
    recordAll(0);
    recordAll(T + 1000, WATCHED.filter((s) => s !== 'battery')); // battery silent for 6 s: still within
    telemetryWatchdog.check([device()], 6000);
    assert.deepEqual(telemetryWatchdog.statusOf(1), {});

    telemetryWatchdog.check([device()], 6001);
    assert.deepEqual(telemetryWatchdog.statusOf(1), { battery: SLOT_HEALTH.STALE });
  } finally {
    delete battery.rate_hz;
  }
});

test('re-emite DEVICE_UPDATED con `telemetry` solo de los devices que cambiaron', () => {
  recordAll(0);
  recordAll(0, WATCHED, 2);
  recordAll(T, WATCHED.filter((s) => s !== 'battery'));
  recordAll(T, WATCHED, 2);
  telemetryWatchdog.check([device(), device({ id: 2, name: 'px4_2' })], T + 1);
  assert.equal(broadcasts.length, 1);
  assert.deepEqual(
    broadcasts[0].map((d) => [d.id, d.telemetry]),
    [[1, { battery: SLOT_HEALTH.STALE }]]
  );
});

test('mide la tasa de cada slot y la expone con su edad', () => {
  for (let t = 0; t < 2000; t += 100) recordAll(t); // 10 Hz during the first 2 s
  telemetryWatchdog.check([device()], 2000);
  const snap = telemetryWatchdog.snapshotOf(1, 2050);
  assert.ok(Math.abs(snap.battery.hz - 10) < 0.01, `hz=${snap.battery.hz}`);
  assert.equal(snap.battery.ageMs, 150);
  assert.equal(snap.battery.health, SLOT_HEALTH.OK);
});

test('la tasa se suaviza entre ventanas en vez de saltar', () => {
  for (let t = 0; t < 2000; t += 100) recordAll(t);
  telemetryWatchdog.check([device()], 2000);
  telemetryWatchdog.check([device()], 4000); // a window with nothing
  assert.ok(Math.abs(telemetryWatchdog.snapshotOf(1, 4000).battery.hz - 5) < 0.01);
});

test('el snapshot incluye los slots marcados aunque nunca hayan llegado', () => {
  recordAll(0, ['position_global']);
  recordAll(T, ['position_global']);
  telemetryWatchdog.check([device()], T + 1);
  const snap = telemetryWatchdog.snapshotOf(1, T + 1);
  assert.deepEqual(snap.battery, { health: SLOT_HEALTH.MISSING, hz: null, ageMs: null });
  assert.deepEqual(telemetryWatchdog.snapshotOf(99), {});
});
