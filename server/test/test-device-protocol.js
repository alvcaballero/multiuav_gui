import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { categoryModel, messageFormatOf, MESSAGE_FORMAT } from '../models/category.js';
import { devicesController } from '../controllers/devices.js';
import { DevicesModel } from '../models/devices.js';
import { serverModel } from '../models/server.js';
import { z } from 'zod';
import { DEVICE_PROTOCOL, resolveAttributeShape } from '../schemas/zod/devices.js';

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
  const base = {};

  test('create refuses a MAVLink category on a ROS device, and the other way', async () => {
    const r1 = await create({ ...base, name: 'p_bad1', category: 'px4_mavlink', protocol: 'ros' });
    assert.equal(r1.statusCode, 400);
    assert.match(r1.body.error[0].message, /speaks MAVLink messages, which protocol 'ros' does not carry/);
    const r2 = await create({ ...base, name: 'p_bad2', category: 'px4_ros2', protocol: 'mavlink', attributes: { systemId: 3 } });
    assert.equal(r2.statusCode, 400);
    assert.deepEqual(r2.body.error[0].path, ['category']);
  });

  test('create accepts a coherent mavlink device', async () => {
    const r = await create({ ...base, name: 'p_ok', category: 'px4_mavlink', protocol: 'mavlink', attributes: { systemId: 4 } });
    assert.equal(r.statusCode, 201);
  });

  test('a partial edit is checked on the device it leaves', async () => {
    await create({ ...base, name: 'p_ros', category: 'px4_ros2', protocol: 'ros' });
    const { id } = await DevicesModel.getByName('p_ros');
    // Only the protocol: the category stays ROS → refused.
    assert.equal((await update(id, { protocol: 'mavlink', attributes: { systemId: 6 } })).statusCode, 400);
    // Protocol + category, but no systemId anywhere → refused.
    const noSysid = await update(id, { protocol: 'mavlink', category: 'px4_mavlink' });
    assert.equal(noSysid.statusCode, 400);
    assert.deepEqual(noSysid.body.error[0].path, ['attributes', 'systemId']);
    // Everything coherent → accepted.
    assert.equal((await update(id, { protocol: 'mavlink', category: 'px4_mavlink', attributes: { systemId: 6 } })).statusCode, 200);
  });

  test('two mavlink devices cannot share a systemId, but a device keeps its own on edit', async () => {
    const mav = { ...base, category: 'px4_mavlink', protocol: 'mavlink' };
    assert.equal((await create({ ...mav, name: 'p_sys_a', attributes: { systemId: 21 } })).statusCode, 201);
    const dup = await create({ ...mav, name: 'p_sys_b', attributes: { systemId: 21 } });
    assert.equal(dup.statusCode, 400);
    assert.deepEqual(dup.body.error[0].path, ['attributes', 'systemId']);
    assert.match(dup.body.error[0].message, /p_sys_a/);

    assert.equal((await create({ ...mav, name: 'p_sys_c', attributes: { systemId: 22 } })).statusCode, 201);
    const { id } = await DevicesModel.getByName('p_sys_c');
    assert.equal((await update(id, { attributes: { systemId: 21 } })).statusCode, 400);
    assert.equal((await update(id, { attributes: { systemId: 22 }, name: 'p_sys_c2' })).statusCode, 200); // its own
    assert.equal((await update(id, { attributes: { systemId: 23 } })).statusCode, 200);
  });

  test('attributes follow the protocol: unknown keys are refused, a protocol change drops the old ones', async () => {
    const r = await create({ ...base, name: 'p_attr_ros', category: 'px4_ros2', protocol: 'ros', attributes: { systemId: 5 } });
    assert.equal(r.statusCode, 400);
    await create({ ...base, name: 'p_attr_mav', category: 'px4_mavlink', protocol: 'mavlink', attributes: { systemId: 31 } });
    const { id } = await DevicesModel.getByName('p_attr_mav');
    // Only the protocol changes: the mavlink attributes are gone, so a ros device keeps none.
    assert.equal((await update(id, { protocol: 'ros', category: 'px4_ros2' })).statusCode, 200);
    assert.deepEqual((await DevicesModel.getById({ id })).attributes, {});
  });

  test('a camera needs the host of its stream, except a Websocket one', async () => {
    const ros = { name: 'p_cam', category: 'px4_ros2', protocol: 'ros' };
    const noHost = await create({ ...ros, camera: [{ type: 'WebRTC', source: 'main' }] });
    assert.equal(noHost.statusCode, 400);
    assert.deepEqual(noHost.body.error[0].path, ['camera', 0, 'host']);
    assert.equal((await create({ ...ros, camera: [{ type: 'Websocket', source: 'main' }] })).statusCode, 201);
    assert.equal((await create({ ...ros, name: 'p_cam2', camera: [{ type: 'WebRTC_env', source: 'main', host: '10.0.0.5' }] })).statusCode, 201);
  });

  test('an unknown device answers 404', async () => {
    assert.equal((await update(999999, { name: 'x' })).statusCode, 404);
  });
});

describe('devicesController.getDevice', () => {
  test('resolves by exact id and never falls back to another device', async () => {
    await devicesController.create({ body: { name: 'p_get', category: 'px4_ros2', protocol: 'ros' } }, fakeRes());
    const { id } = await DevicesModel.getByName('p_get');
    assert.equal((await devicesController.getDevice(id)).name, 'p_get');
    assert.equal((await devicesController.getDevice(String(id))).id, id);
    // getAll answers every device for these, and the old `.at()` returned the first one
    for (const bad of [undefined, null, 'abc', 0, 999999]) {
      assert.equal(await devicesController.getDevice(bad), null, String(bad));
    }
  });
});

describe('attribute groups', () => {
  const groups = [
    { id: 'mav', appliesTo: ({ protocol }) => protocol === 'mavlink', shape: { systemId: z.number() } },
    { id: 'by-category', appliesTo: ({ category }) => category === 'px4_mavlink', shape: { px4Only: z.string() } },
    // depends on ANOTHER attribute's value, not on the protocol or the category
    { id: 'by-value', appliesTo: ({ attributes }) => attributes.systemId === 1, shape: { primary: z.boolean() } },
  ];
  const keys = (device) => Object.keys(resolveAttributeShape(groups, device)).sort();

  test('the keys of a device are the union of the groups that apply to it', () => {
    assert.deepEqual(keys({ protocol: 'ros', category: 'px4_ros2' }), []);
    assert.deepEqual(keys({ protocol: 'mavlink', category: 'ardupilot_mavlink' }), ['systemId']);
    assert.deepEqual(keys({ protocol: 'mavlink', category: 'px4_mavlink' }), ['px4Only', 'systemId']);
    assert.deepEqual(keys({ protocol: 'mavlink', category: 'px4_mavlink', attributes: { systemId: 1 } }), ['primary', 'px4Only', 'systemId']);
  });

  test('two groups declaring the same key for one device throw', () => {
    const clash = [...groups, { id: 'dup', appliesTo: () => true, shape: { systemId: z.number() } }];
    assert.throws(() => resolveAttributeShape(clash, { protocol: 'mavlink' }), /declared twice/);
  });
});
