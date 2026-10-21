import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { categoryModel, messageFormatOf, MESSAGE_FORMAT } from '../models/category.js';
import { devicesController } from '../controllers/devices.js';
import { DevicesModel } from '../models/devices.js';
import { serverModel } from '../models/server.js';
import { DEVICE_PROTOCOL } from '../schemas/zod/devices.js';

// Which categories a protocol can use is derived from the category's own messages (mavlink/* or
// ROS-shaped), and the device API refuses a combination that would leave the device deaf.

const fakeRes = () => {
  const res = { statusCode: 200, body: undefined };
  res.status = (code) => ((res.statusCode = code), res);
  res.json = (body) => ((res.body = body), res);
  return res;
};

// The protocol (ros, mqtt, robofleet, mavlink) picks the gateway; the category only says which
// message format it speaks, and that decides which protocols can carry it.
describe('category message format', () => {
  test('derived from the messages: mavlink/* is MAVLink, the rest ROS-shaped', () => {
    assert.equal(messageFormatOf(categoryModel.getCategory('px4_mavlink')), MESSAGE_FORMAT.MAVLINK);
    assert.equal(messageFormatOf(categoryModel.getCategory('ardupilot_mavlink')), MESSAGE_FORMAT.MAVLINK);
    assert.equal(messageFormatOf(categoryModel.getCategory('px4_ros2')), MESSAGE_FORMAT.ROS_MSG);
    assert.equal(messageFormatOf(categoryModel.getCategory('simulationFB')), MESSAGE_FORMAT.ROS_MSG);
  });

  test('getAll by protocol: mavlink lists the MAVLink categories; ros, mqtt and robofleet the others', () => {
    const mav = categoryModel.getAll({ protocol: 'mavlink' });
    assert.deepEqual(mav.sort(), ['ardupilot_mavlink', 'px4_mavlink']);
    for (const protocol of ['ros', 'mqtt', 'robofleet']) {
      const list = categoryModel.getAll({ protocol });
      assert.ok(list.includes('px4_ros2') && !list.includes('px4_mavlink'), protocol);
    }
    assert.equal(categoryModel.getAll().length, categoryModel.getAll({ protocol: 'ros' }).length + mav.length);
  });
});

describe('GET /api/server/protocol', () => {
  // All four always, so the form shows the disabled ones greyed out instead of hiding them.
  test('lists every device protocol, each with whether this server runs it', () => {
    const list = serverModel.Protocol();
    assert.deepEqual(list.map((p) => p.name).sort(), [...DEVICE_PROTOCOL.options].sort());
    assert.ok(list.every((p) => typeof p.enabled === 'boolean'));
  });
});

describe('device API: protocol ↔ category', () => {
  const create = async (body) => {
    const res = fakeRes();
    await devicesController.create({ body }, res);
    return res;
  };
  const update = async (id, body) => {
    const res = fakeRes();
    await devicesController.update({ params: { id }, body }, res);
    return res;
  };
  const base = { ip: '127.0.0.1' };

  test('create refuses a MAVLink category on a ROS device, and the other way', async () => {
    const r1 = await create({ ...base, name: 'p_bad1', category: 'px4_mavlink', protocol: 'ros' });
    assert.equal(r1.statusCode, 400);
    assert.match(r1.body.error[0].message, /speaks MAVLink messages, which protocol 'ros' does not carry/);
    const r2 = await create({ ...base, name: 'p_bad2', category: 'px4_ros2', protocol: 'mavlink', systemId: 3 });
    assert.equal(r2.statusCode, 400);
    assert.deepEqual(r2.body.error[0].path, ['category']);
  });

  test('create accepts a coherent mavlink device', async () => {
    const r = await create({ ...base, name: 'p_ok', category: 'px4_mavlink', protocol: 'mavlink', systemId: 4 });
    assert.equal(r.statusCode, 201);
  });

  test('a partial edit is checked on the device it leaves', async () => {
    await create({ ...base, name: 'p_ros', category: 'px4_ros2', protocol: 'ros' });
    const { id } = await DevicesModel.getByName('p_ros');
    // Only the protocol: the category stays ROS → refused.
    assert.equal((await update(id, { protocol: 'mavlink', systemId: 6 })).statusCode, 400);
    // Protocol + category, but no systemId anywhere → refused.
    const noSysid = await update(id, { protocol: 'mavlink', category: 'px4_mavlink' });
    assert.equal(noSysid.statusCode, 400);
    assert.deepEqual(noSysid.body.error[0].path, ['systemId']);
    // Everything coherent → accepted.
    assert.equal((await update(id, { protocol: 'mavlink', category: 'px4_mavlink', systemId: 6 })).statusCode, 200);
  });

  test('two mavlink devices cannot share a systemId, but a device keeps its own on edit', async () => {
    const mav = { ...base, category: 'px4_mavlink', protocol: 'mavlink' };
    assert.equal((await create({ ...mav, name: 'p_sys_a', systemId: 21 })).statusCode, 201);
    const dup = await create({ ...mav, name: 'p_sys_b', systemId: 21 });
    assert.equal(dup.statusCode, 400);
    assert.deepEqual(dup.body.error[0].path, ['systemId']);
    assert.match(dup.body.error[0].message, /p_sys_a/);

    assert.equal((await create({ ...mav, name: 'p_sys_c', systemId: 22 })).statusCode, 201);
    const { id } = await DevicesModel.getByName('p_sys_c');
    assert.equal((await update(id, { systemId: 21 })).statusCode, 400);
    assert.equal((await update(id, { systemId: 22, name: 'p_sys_c2' })).statusCode, 200); // its own
    assert.equal((await update(id, { systemId: 23 })).statusCode, 200);
  });

  test('an unknown device answers 404', async () => {
    assert.equal((await update(999999, { name: 'x' })).statusCode, 404);
  });
});
