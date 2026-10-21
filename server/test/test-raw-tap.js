import { test, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { setTimeout as sleep } from 'node:timers/promises';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { rawTap, RAW_REJECT } from '../models/inspect/rawTap.js';
import { WebSocketSubscriber } from '../subscribers/websocketSubscriber.js';
import { WS_RAW_INSPECT_INTERVAL_MS } from '../config/config.js';

const raw = (over = {}) => ({ deviceId: 1, slot: 'position', transport: 'ros', raw: { n: 0 }, ...over });
const afterWindow = () => sleep(WS_RAW_INSPECT_INTERVAL_MS + 60);

let emitted;
eventBus.on(EVENTS.RAW_MESSAGE, (msg) => emitted.push(msg));

beforeEach(() => {
  emitted = [];
  for (const clientId of [...rawTap._byClient.keys()]) rawTap.removeClient(clientId);
});

test('sin suscriptores no emite nada', () => {
  rawTap.publish(raw());
  assert.equal(emitted.length, 0);
});

test('emite solo al cliente suscripto al device y slot', () => {
  rawTap.subscribe('c1', 1, 'position');
  rawTap.publish(raw({ slot: 'battery' }));
  rawTap.publish(raw({ deviceId: 2 }));
  assert.equal(emitted.length, 0);

  rawTap.publish(raw({ raw: { latitude: 1 } }));
  assert.equal(emitted.length, 1);
  assert.equal(emitted[0].clientId, 'c1');
  assert.deepEqual(emitted[0].data, { latitude: 1 });
});

test('dentro de la ventana el primero sale ya y el ÚLTIMO sale al cerrarla; los del medio se descartan', async () => {
  rawTap.subscribe('c1', 1, 'position');
  rawTap.publish(raw({ raw: { n: 1 } }));
  rawTap.publish(raw({ raw: { n: 2 } }));
  rawTap.publish(raw({ raw: { n: 3 } }));
  assert.deepEqual(emitted.map((e) => e.data.n), [1]);

  await afterWindow();
  assert.deepEqual(emitted.map((e) => e.data.n), [1, 3], 'el cambio más reciente no se pierde');
});

test('un mensaje solo, sin nada que lo siga, sale inmediato y no deja timers', async () => {
  rawTap.subscribe('c1', 1, 'position');
  rawTap.publish(raw({ raw: { n: 1 } }));
  await afterWindow();
  assert.equal(emitted.length, 1);
});

test('el muestreo es por cliente', () => {
  rawTap.subscribe('c1', 1, 'position');
  rawTap.publish(raw());
  rawTap.subscribe('c2', 1, 'position');
  rawTap.publish(raw());
  assert.deepEqual(emitted.map((e) => e.clientId), ['c1', 'c2']);
});

test('unsubscribe y removeClient dejan de emitir, también lo que estaba pendiente', async () => {
  rawTap.subscribe('c1', 1, 'position');
  rawTap.publish(raw({ raw: { n: 1 } }));
  rawTap.publish(raw({ raw: { n: 2 } })); // pendiente
  rawTap.unsubscribe('c1', 1, 'position');
  await afterWindow();
  assert.equal(emitted.length, 1, 'el pendiente no se envía a un cliente que ya se fue');
  assert.equal(rawTap._subs.size, 0);
  assert.equal(rawTap._byClient.size, 0);

  rawTap.subscribe('c1', 1, 'position');
  rawTap.subscribe('c1', 1, 'battery');
  rawTap.removeClient('c1');
  assert.equal(rawTap._subs.size, 0);
  assert.equal(rawTap._byClient.size, 0);
  rawTap.publish(raw());
  assert.equal(emitted.length, 1);
});

test('subscribe devuelve el motivo cuando rechaza', () => {
  assert.deepEqual(rawTap.subscribe('c1', 1, 'position'), { ok: true });
  assert.deepEqual(rawTap.subscribe('c1', 1, 'camera'), { ok: false, reason: RAW_REJECT.NOT_INSPECTABLE });
  for (const args of [['c1', undefined, 'position'], ['c1', 1, ''], [undefined, 1, 'position']]) {
    assert.deepEqual(rawTap.subscribe(...args), { ok: false, reason: RAW_REJECT.INVALID }, String(args));
  }
});

test('un cliente no puede abrir suscripciones sin límite, pero repetir una no cuenta', () => {
  for (let i = 0; i < 8; i++) assert.equal(rawTap.subscribe('c1', i, 'position').ok, true);
  assert.deepEqual(rawTap.subscribe('c1', 99, 'position'), { ok: false, reason: RAW_REJECT.TOO_MANY });
  assert.equal(rawTap.subscribe('c1', 3, 'position').ok, true, 'ya la tenía');
  assert.equal(rawTap.subscribe('c2', 99, 'position').ok, true, 'el tope es por cliente');
});

test('raw como función: no se ejecuta sin suscriptores', () => {
  let calls = 0;
  rawTap.publish(raw({ raw: () => (calls++, { ok: true }) }));
  assert.equal(calls, 0);
});

test('raw como función: una vez por mensaje aunque haya dos clientes, y nunca para un mensaje pisado', async () => {
  const calls = [];
  const lazy = (n) => raw({ raw: () => (calls.push(n), { n }) });

  rawTap.subscribe('c1', 1, 'position');
  rawTap.subscribe('c2', 1, 'position');
  rawTap.publish(lazy(1));
  assert.deepEqual(calls, [1], 'una vez para los dos');
  assert.equal(emitted.length, 2);

  rawTap.publish(lazy(2)); // queda pendiente...
  rawTap.publish(lazy(3)); // ...y este lo pisa antes de enviarse
  assert.deepEqual(calls, [1], 'ninguno se desempaqueta todavía');

  await afterWindow();
  assert.deepEqual(calls, [1, 3], 'solo el último se desempaqueta, una vez para los dos clientes');
  assert.deepEqual(emitted.map((e) => e.data.n), [1, 1, 3, 3]);
});

test('si la función de raw falla, no se emite y no se propaga', () => {
  rawTap.subscribe('c1', 1, 'position');
  rawTap.publish(raw({ raw: () => { throw new Error('boom'); } }));
  assert.equal(emitted.length, 0);
});

test('el subscriber manda RAW_MESSAGE y RAW_STATUS solo al cliente destino, no broadcast', async () => {
  const sent = { unicast: [], broadcast: [] };
  const sub = new WebSocketSubscriber({
    sendToClient: (id, msg) => sent.unicast.push({ id, msg }),
    sendMessage: (msg) => sent.broadcast.push(msg),
  });
  try {
    eventBus.emit(EVENTS.RAW_MESSAGE, { clientId: 'c9', deviceId: 1, slot: 'position', transport: 'ros', ts: 1, data: { a: 1 } });
    eventBus.emit(EVENTS.RAW_STATUS, { clientId: 'c9', deviceId: 1, slot: 'camera', ok: false, reason: 'not_inspectable' });
    await new Promise((r) => setImmediate(r));

    assert.equal(sent.broadcast.length, 0);
    assert.deepEqual(sent.unicast, [
      { id: 'c9', msg: { raw: { deviceId: 1, slot: 'position', transport: 'ros', ts: 1, data: { a: 1 } } } },
      { id: 'c9', msg: { rawStatus: { deviceId: 1, slot: 'camera', ok: false, reason: 'not_inspectable' } } },
    ]);
  } finally {
    sub.cleanup();
  }
});

test('guarda el último mensaje sin suscriptores y se lo manda al que se suscribe, marcado cached', async () => {
  rawTap._last.clear();
  rawTap.publish(raw({ slot: 'vehicle_command_ack', raw: { n: 1 } }));
  rawTap.publish(raw({ slot: 'vehicle_command_ack', raw: { n: 2 } }));
  assert.equal(emitted.length, 0);

  await sleep(20);
  rawTap.subscribe('c1', 1, 'vehicle_command_ack');
  rawTap.replayLast('c1', 1, 'vehicle_command_ack');
  assert.equal(emitted.length, 1);
  assert.deepEqual(emitted[0].data, { n: 2 }, 'the newest one');
  assert.equal(emitted[0].cached, true);
  assert.ok(emitted[0].ageMs >= 20, `ageMs=${emitted[0].ageMs}`);

  rawTap.publish(raw({ slot: 'vehicle_command_ack', raw: { n: 3 } }));
  await afterWindow();
  assert.equal(emitted.at(-1).cached, undefined, 'live messages are not marked');
});

test('replayLast no manda nada sin suscripción, sin caché, ni para la cámara', () => {
  rawTap._last.clear();
  rawTap.publish(raw({ slot: 'camera' }));
  rawTap.publish(raw({ slot: 'battery' }));
  rawTap.replayLast('c1', 1, 'battery'); // not subscribed
  rawTap.subscribe('c1', 1, 'gimbal');
  rawTap.replayLast('c1', 1, 'gimbal'); // nothing stored
  assert.equal(emitted.length, 0);
  assert.equal(rawTap._last.has('1:camera'), false);
});

test('el caché resuelve un raw perezoso una sola vez y suelta el thunk', () => {
  rawTap._last.clear();
  let calls = 0;
  rawTap.publish(raw({ slot: 'battery', raw: () => (calls++, { v: 1 }) }));
  for (const c of ['c1', 'c2']) {
    rawTap.subscribe(c, 1, 'battery');
    rawTap.replayLast(c, 1, 'battery');
  }
  assert.equal(calls, 1);
  assert.equal(emitted.length, 2);
  assert.equal(rawTap._last.get('1:battery').raw, undefined);
});
