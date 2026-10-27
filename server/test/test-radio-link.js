import { test, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { eventBus, EVENTS } from '../common/eventBus.js';
import { parseStations } from '../models/radio/ubiquitiSnmp.js';
import { LINK_LEVEL, findStation, hasRadio, levelOfSnr, linkOfStation } from '../models/radio/linkQuality.js';
import { radioLinkMonitor } from '../models/radio/radioLinkMonitor.js';
import { validateDeviceAttributes } from '../schemas/zod/devices.js';
import { RADIO_POLL_INTERVAL_MS } from '../config/config.js';

const ENTRY = '1.3.6.1.4.1.41112.1.4.7.1';
const MAC_INDEX = '1.172.139.169.192.161.203'; // wlan 1 + ac:8b:a9:c0:a1:cb

// The station table of a Bullet AC as the AP answered it (AirOS 8, SNMPv1), at a given signal.
const row = (index, { signal = -38, noise = -87, ip = '10.20.0.64' } = {}) =>
  [
    [2, Buffer.from('Bullet AC')],
    [3, signal],
    [4, noise],
    [5, 300],
    [6, 333],
    [10, ip],
    [11, 200000000],
    [12, 200000000],
    [13, Buffer.from([0xaf, 0x00])], // Counter64 over SNMPv1: garbage, must be ignored
    [15, 24900],
    [16, 26],
    [17, 169200],
    [18, 140400],
  ].map(([column, value]) => ({ oid: `${ENTRY}.${column}.${index}`, value }));

const station = (over) => parseStations(row(MAC_INDEX, over))[0];

test('parseStations: lee la fila de la tabla de estaciones del AP', () => {
  const [sta] = parseStations(row(MAC_INDEX));
  assert.equal(sta.mac, 'ac:8b:a9:c0:a1:cb');
  assert.equal(sta.name, 'Bullet AC');
  assert.equal(sta.ip, '10.20.0.64');
  assert.equal(sta.signal, -38);
  assert.equal(sta.noise, -87);
  assert.equal(sta.txRate, 200000000);
  assert.equal(sta.txCapacity, 169200);
  assert.equal('ccq' in sta, false);
});

test('parseStations: una fila por MAC, y descarta las que no traen señal o ruido', () => {
  const other = '1.0.0.0.0.0.2';
  const half = row(other).filter((vb) => !vb.oid.includes('.3.')); // no signal
  assert.equal(parseStations([...row(MAC_INDEX), ...half]).length, 1);
  assert.equal(parseStations([...row(MAC_INDEX), ...row('1.1.2.3.4.5.6')]).length, 2);
});

test('parseStations: ignora OIDs ajenos a la tabla', () => {
  assert.deepEqual(parseStations([{ oid: '1.3.6.1.2.1.1.1.0', value: 'Linux' }]), []);
});

test('linkOfStation: SNR y unidades (bit/s → Mbps, kbit/s → Mbps)', () => {
  const link = linkOfStation(station());
  assert.equal(link.snr, 49);
  assert.equal(link.txRate, 200);
  assert.equal(link.rxCapacity, 140.4);
});

test('levelOfSnr: umbrales y histéresis', () => {
  assert.equal(levelOfSnr(30), LINK_LEVEL.GOOD);
  assert.equal(levelOfSnr(20), LINK_LEVEL.FAIR);
  assert.equal(levelOfSnr(10), LINK_LEVEL.POOR);
  // sobre el límite no oscila: good aguanta hasta 23, fair hasta 13
  assert.equal(levelOfSnr(24, LINK_LEVEL.GOOD), LINK_LEVEL.GOOD);
  assert.equal(levelOfSnr(22, LINK_LEVEL.GOOD), LINK_LEVEL.FAIR);
  assert.equal(levelOfSnr(14, LINK_LEVEL.FAIR), LINK_LEVEL.FAIR);
  assert.equal(levelOfSnr(12, LINK_LEVEL.FAIR), LINK_LEVEL.POOR);
  // subir no tiene histéresis, y desde lost/unknown vale el nivel crudo
  assert.equal(levelOfSnr(25, LINK_LEVEL.FAIR), LINK_LEVEL.GOOD);
  assert.equal(levelOfSnr(24, LINK_LEVEL.LOST), LINK_LEVEL.FAIR);
});

test('findStation: por MAC, y por IP solo si no hay MAC', () => {
  const stations = [station()];
  assert.equal(findStation(stations, { radioMac: 'ac:8b:a9:c0:a1:cb' }).name, 'Bullet AC');
  assert.equal(findStation(stations, { radioIp: '10.20.0.64' }).name, 'Bullet AC');
  assert.equal(findStation(stations, { radioMac: 'aa:aa:aa:aa:aa:aa', radioIp: '10.20.0.64' }), null);
  assert.equal(findStation(stations, {}), null);
  assert.equal(hasRadio({ radioIp: '10.20.0.64' }), true);
  assert.equal(hasRadio({ systemId: 1 }), false);
});

test('atributos radioIp / radioMac: se validan y la MAC se normaliza', () => {
  const dev = { protocol: 'ros', category: 'px4_ros2' };
  const ok = validateDeviceAttributes(dev, { radioIp: '10.20.0.64', radioMac: 'AC-8B-A9-C0-A1-CB' });
  assert.equal(ok.success, true);
  assert.equal(ok.data.radioMac, 'ac:8b:a9:c0:a1:cb');
  assert.equal(validateDeviceAttributes(dev, { radioIp: '10.20.0.300' }).success, false);
  assert.equal(validateDeviceAttributes(dev, { radioMac: 'nope' }).success, false);
  assert.equal(validateDeviceAttributes(dev, { radioIp: null, radioMac: null }).success, true); // null clears
});

// --- monitor ---------------------------------------------------------------------------------------------
const LOST_AFTER = 3 * RADIO_POLL_INTERVAL_MS;
const device = (over = {}) => ({ id: 7, name: 'x500_1', attributes: { radioIp: '10.20.0.64' }, ...over });

let events;
let broadcasts;
eventBus.on(EVENTS.DEVICE_UPDATED, (devices) => broadcasts.push(devices));

beforeEach(() => {
  events = [];
  broadcasts = [];
  radioLinkMonitor._links.clear();
  radioLinkMonitor._apOkAt = null;
  radioLinkMonitor._addEvent = (e) => events.push(e);
});

test('un device sin radio configurado no se sigue (statusOf → null)', () => {
  radioLinkMonitor.check([device({ attributes: {} })], [station()], 0);
  assert.equal(radioLinkMonitor.statusOf(7), null);
});

test('enlace bueno: nivel, métricas y DEVICE_UPDATED al cambiar; sin Event', () => {
  const changed = radioLinkMonitor.check([device()], [station()], 1000);
  assert.equal(changed.length, 1);
  const status = radioLinkMonitor.statusOf(7, 1500);
  assert.equal(status.level, LINK_LEVEL.GOOD);
  assert.equal(status.snr, 49);
  assert.equal(status.ageMs, 500);
  assert.equal(broadcasts.length, 1);
  assert.equal(broadcasts[0][0].link.level, LINK_LEVEL.GOOD);
  assert.equal(events.length, 0);
  // mismo nivel en el poll siguiente: nada que reportar
  assert.equal(radioLinkMonitor.check([device()], [station({ signal: -40 })], 2000).length, 0);
  assert.equal(broadcasts.length, 1);
});

test('enlace pobre avisa con warning y al volver con info', () => {
  radioLinkMonitor.check([device()], [station()], 0);
  radioLinkMonitor.check([device()], [station({ signal: -80 })], 1000); // snr 7
  assert.equal(radioLinkMonitor.statusOf(7).level, LINK_LEVEL.POOR);
  assert.deepEqual(
    events.map((e) => e.type),
    ['warning']
  );
  radioLinkMonitor.check([device()], [station()], 2000);
  assert.deepEqual(
    events.map((e) => e.type),
    ['warning', 'info']
  );
});

test('la estación desaparece de la tabla: lost solo pasada la tolerancia, y vuelve', () => {
  radioLinkMonitor.check([device()], [station()], 0);
  radioLinkMonitor.check([device()], [], LOST_AFTER); // no pasó de la tolerancia
  assert.equal(radioLinkMonitor.statusOf(7).level, LINK_LEVEL.GOOD);
  radioLinkMonitor.check([device()], [], LOST_AFTER + 1);
  assert.equal(radioLinkMonitor.statusOf(7).level, LINK_LEVEL.LOST);
  assert.equal(events.at(-1).type, 'warning');
  radioLinkMonitor.check([device()], [station()], LOST_AFTER + 2000);
  assert.equal(radioLinkMonitor.statusOf(7).level, LINK_LEVEL.GOOD);
  assert.equal(events.at(-1).type, 'info');
});

test('un drone apagado al arrancar queda lost sin Event', () => {
  radioLinkMonitor.check([device()], [], 0);
  assert.equal(radioLinkMonitor.statusOf(7).level, LINK_LEVEL.LOST);
  assert.equal(events.length, 0);
});

test('el AP no contesta: unknown pasada la tolerancia, sin Event, y no es lost', () => {
  radioLinkMonitor.check([device()], [station()], 0);
  radioLinkMonitor.check([device()], null, LOST_AFTER);
  assert.equal(radioLinkMonitor.statusOf(7).level, LINK_LEVEL.GOOD);
  radioLinkMonitor.check([device()], null, LOST_AFTER + 1);
  assert.equal(radioLinkMonitor.statusOf(7).level, LINK_LEVEL.UNKNOWN);
  assert.equal(events.length, 0);
  assert.equal(radioLinkMonitor.statusOf(7).snr, 49); // keeps the last reading
});

test('un device al que se le quita el radio deja de seguirse', () => {
  radioLinkMonitor.check([device()], [station()], 0);
  radioLinkMonitor.check([device({ attributes: {} })], [station()], 1000);
  assert.equal(radioLinkMonitor.statusOf(7), null);
});
