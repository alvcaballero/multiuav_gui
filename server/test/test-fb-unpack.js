import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as fb from 'fbmsglib';
import * as flatbuffers from 'flatbuffers';
import { unpackFbMsg, toJsonSafe } from '../models/flatbuffer/fbUnpack.js';
import { slotFromTopic, slotOfTopic } from '../models/flatbuffer/fbDecode.js';
import { categoryModel } from '../models/category.js';

function float64Buffer(data) {
  const t = new fb.fb.std_msgs.Float64T();
  t.data = data;
  const b = new flatbuffers.Builder(64);
  b.finish(t.pack(b));
  return new flatbuffers.ByteBuffer(b.asUint8Array());
}

test('unpackFbMsg resuelve la clase por el type, sin tabla', () => {
  const out = unpackFbMsg('std_msgs/Float64', float64Buffer(3.14));
  assert.equal(out.data, 3.14);
});

test('unpackFbMsg acepta un type estilo ROS2 (pkg/msg/Name)', () => {
  assert.equal(unpackFbMsg('std_msgs/msg/Float64', float64Buffer(2.5)).data, 2.5);
});

test('un type que fbmsglib no conoce devuelve hex, no revienta', () => {
  const out = unpackFbMsg('px4_msgs/VehicleStatus', float64Buffer(1));
  assert.equal(out.unsupported, 'px4_msgs/VehicleStatus');
  assert.ok(out.size > 0);
  assert.match(out.hex, /^[0-9a-f]+$/);
});

test('un type hostil no sale por el prototipo', () => {
  for (const type of ['__proto__/x', 'constructor/name', 'std_msgs/__proto__', '', undefined]) {
    assert.ok(unpackFbMsg(type, float64Buffer(1)).unsupported !== undefined, String(type));
  }
});

test('toJsonSafe resume los vectores de bytes largos y deja los cortos', () => {
  const out = toJsonSafe({ small: new Uint8Array([1, 2, 3]), big: new Uint8Array(1000), n: 7 });
  assert.deepEqual(out.small, [1, 2, 3]);
  assert.equal(out.big, '<1000 items>');
  assert.equal(out.n, 7);
});

test('slotOfTopic: la categoría manda; lo que no declara conserva su nombre', () => {
  const subs = { position: { name: '/gps_position' }, speed: { name: '/speed' } };
  assert.equal(slotOfTopic(subs, '/uav1/gps_position'), 'position');
  assert.equal(slotOfTopic(subs, 'uav1/speed'), 'speed');
  assert.equal(slotOfTopic(subs, '/uav1/dji_extra'), 'dji_extra');
  assert.equal(slotOfTopic(undefined, '/uav1/gps_position'), 'gps_position');
});

test('simulationFB: sus topics resuelven a los slots que fbDecode entiende', () => {
  const subs = categoryModel.getCategory('simulationFB').subscribers;
  assert.equal(slotOfTopic(subs, '/uav1/gps_position'), 'position');
  assert.equal(slotOfTopic(subs, '/uav1/orientation'), 'heading');
  assert.equal(slotOfTopic(subs, '/uav1/alarm'), 'threat');
});

test('slotFromTopic quita el prefijo del device', () => {
  assert.equal(slotFromTopic('/uav1/gps_position'), 'gps_position');
  assert.equal(slotFromTopic('uav1/gps_position'), 'gps_position');
  assert.equal(slotFromTopic('/uav1/gimbal/angle'), 'gimbal/angle');
});
